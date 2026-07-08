// Copyright 2026 Boombroke
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include "sentry_decision/decision_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

namespace sentry_decision
{

DecisionNode::DecisionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("sentry_decision", options)
{
  const std::string profile_path = this->declare_parameter<std::string>("profile_path", "");
  const double tick_hz = this->declare_parameter<double>("tick_frequency", 10.0);
  const std::string goal_topic = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
  const std::string auto_aim_target_topic =
    this->declare_parameter<std::string>("auto_aim_target_topic", "auto_aim_target_pos");
  nav_action_name_ = this->declare_parameter<std::string>("nav_action_name", "navigate_to_pose");
  goal_reached_distance_tolerance_ =
    this->declare_parameter<double>("goal_reached_distance_tolerance", 0.25);
  goal_frame_ = this->declare_parameter<std::string>("goal_frame", "map");

  if (profile_path.empty()) {
    throw std::runtime_error("sentry_decision: 'profile_path' parameter is required");
  }

  Profile profile = load_profile(profile_path);
  context_.set_thresholds(profile.thresholds);
  RCLCPP_INFO(
    get_logger(), "loaded decision profile '%s' (%zu patrol, %zu attack, %zu defend wp)",
    profile.name.c_str(), profile.patrol.size(), profile.attack_push.size(),
    profile.defend_fallback.size());

  pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
  pub_sentry_cmd_ = this->create_publisher<rm_interfaces::msg::SentryCommand>("sentry/command", 10);
  nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);

  fsm_ = std::make_unique<DecisionFsm>(
    std::move(profile), [this](const Waypoint & wp) { publish_goal(wp); },
    [this](StanceCommand stance) { send_stance_command(stance); }, [this]() { cancel_nav(); });

  sub_game_status_ = create_subscription<rm_interfaces::msg::GameStatus>(
    "referee/game_status", 10,
    [this](const rm_interfaces::msg::GameStatus::SharedPtr m) { context_.update(*m); });
  sub_robot_status_ = create_subscription<rm_interfaces::msg::RobotStatus>(
    "referee/robot_status", 10,
    [this](const rm_interfaces::msg::RobotStatus::SharedPtr m) { context_.update(*m); });
  sub_rfid_status_ = create_subscription<rm_interfaces::msg::RfidStatus>(
    "referee/rfidStatus", 10,
    [this](const rm_interfaces::msg::RfidStatus::SharedPtr m) { context_.update(*m); });
  sub_robot_hp_ = create_subscription<rm_interfaces::msg::GameRobotHP>(
    "referee/all_robot_hp", 10,
    [this](const rm_interfaces::msg::GameRobotHP::SharedPtr m) { context_.update(*m); });
  sub_auto_aim_target_ = create_subscription<std_msgs::msg::String>(
    auto_aim_target_topic, 10,
    std::bind(&DecisionNode::auto_aim_target_callback, this, std::placeholders::_1));
  sub_radar_ = create_subscription<radar_msgs::msg::EnemyPosition>(
    "radar/enemy_positions", 10,
    std::bind(&DecisionNode::radar_callback, this, std::placeholders::_1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry", 10, std::bind(&DecisionNode::odometry_callback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / tick_hz);
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { on_tick(); });

  RCLCPP_INFO(get_logger(), "sentry_decision started @ %.1f Hz", tick_hz);
  RCLCPP_INFO(
    get_logger(), "auto aim target input: %s (std_msgs/String x,y,valid,id)",
    auto_aim_target_topic.c_str());
}

void DecisionNode::on_tick()
{
  const double now_s = this->now().seconds();
  context_.set_now(now_s);

  // Fallback 模式：无 Nav2 到达回调时，自己用 odom 算距离
  if (fallback_goal_active_ && nav_goals_active_) {
    const double dx = current_x_ - fallback_goal_x_;
    const double dy = current_y_ - fallback_goal_y_;
    const double dist = std::hypot(dx, dy);
    if (dist <= goal_reached_distance_tolerance_) {
      context_.set_nav_status(NavStatus::ARRIVED);
      fallback_goal_active_ = false;
    }
  }

  if (!context_.has_referee()) {
    return;
  }

  fsm_->tick(context_, now_s);
  const State current = fsm_->state();

  if (current == State::IDLE && nav_goals_active_) {
    nav_goals_active_ = false;
    fallback_goal_active_ = false;
  }

  if (current != last_logged_state_ || fsm_->combat_substate() != last_logged_combat_substate_) {
    RCLCPP_INFO(
      get_logger(), "state -> %s/%s (hp=%u ammo=%u remain=%ds nav=%d)", to_string(current),
      to_string(fsm_->combat_substate()), context_.hp(), context_.ammo(), context_.remain_time(),
      static_cast<int>(context_.nav_status()));
    last_logged_state_ = current;
    last_logged_combat_substate_ = fsm_->combat_substate();
  }
}

void DecisionNode::publish_goal(const Waypoint & wp)
{
  // 新 goal 到来时清掉旧的 fallback 坐标，防止 on_tick 误判到达
  fallback_goal_active_ = false;

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.header.frame_id = goal_frame_;
  goal_msg.pose.pose.position.x = wp.x;
  goal_msg.pose.pose.position.y = wp.y;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.w = 1.0;

  if (!nav_action_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Nav2 action '%s' not ready; falling back to %s PoseStamped", nav_action_name_.c_str(),
      pub_goal_->get_topic_name());
    publish_goal_topic_fallback(wp);
    return;
  }

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback = [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
    goal_response_callback(goal_handle);
  };
  options.feedback_callback = [this](
                                GoalHandleNavigateToPose::SharedPtr goal_handle,
                                const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    feedback_callback(goal_handle, feedback);
  };
  options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
    result_callback(result);
  };

  (void)nav_action_client_->async_send_goal(goal_msg, options);
  nav_goals_active_ = true;
  context_.set_nav_status(NavStatus::MOVING);
  RCLCPP_DEBUG(get_logger(), "action goal -> (%.2f, %.2f)", wp.x, wp.y);
}

void DecisionNode::publish_goal_topic_fallback(const Waypoint & wp)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = this->now();
  goal.header.frame_id = goal_frame_;
  goal.pose.position.x = wp.x;
  goal.pose.position.y = wp.y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;
  pub_goal_->publish(goal);
  nav_goals_active_ = true;
  fallback_goal_x_ = wp.x;
  fallback_goal_y_ = wp.y;
  fallback_goal_active_ = true;
  context_.set_nav_status(NavStatus::MOVING);
  RCLCPP_DEBUG(get_logger(), "fallback goal -> (%.2f, %.2f)", wp.x, wp.y);
}

void DecisionNode::cancel_nav()
{
  if (!nav_goals_active_) {
    return;
  }
  if (!nav_action_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Nav2 action server not available, cannot cancel goals");
    // fallback 模式下仍需清理标志位，防止 on_tick 用旧坐标做到达检测
    nav_goals_active_ = false;
    fallback_goal_active_ = false;
    context_.set_nav_status(NavStatus::IDLE);
    return;
  }
  // 始终取消所有 goal：current_goal_handle_ 可能指向旧 goal，
  // 而新 goal 已在 route 推进时发送但 goal_response 尚未更新 handle。
  (void)nav_action_client_->async_cancel_all_goals();
  current_goal_handle_.reset();
  nav_goals_active_ = false;
  context_.set_nav_status(NavStatus::IDLE);
  RCLCPP_INFO(get_logger(), "cancelled all Nav2 goals");
}

void DecisionNode::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    context_.set_nav_status(NavStatus::FAILED);
    nav_goals_active_ = false;
    RCLCPP_WARN(get_logger(), "Nav2 rejected the decision goal");
    return;
  }
  current_goal_handle_ = goal_handle;
  context_.set_nav_status(NavStatus::MOVING);
}

void DecisionNode::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr goal_handle,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  // 用 goal_id 比较而非 shared_ptr 地址比较，与 result_callback 保持一致
  if (current_goal_handle_ && goal_handle->get_goal_id() != current_goal_handle_->get_goal_id()) {
    return;
  }
  if (feedback->distance_remaining <= goal_reached_distance_tolerance_) {
    context_.set_nav_status(NavStatus::ARRIVED);
    return;
  }
  context_.set_nav_status(NavStatus::MOVING);
}

void DecisionNode::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  // 过滤旧 goal 的延迟回调：若当前活跃 goal handle 与回调 goal_id 不一致，
  // 说明新 goal 已发送，忽略此 stale 结果以防覆盖 nav_status。
  if (current_goal_handle_ && result.goal_id != current_goal_handle_->get_goal_id()) {
    RCLCPP_DEBUG(get_logger(), "ignoring stale Nav2 result for a superseded goal");
    return;
  }
  current_goal_handle_.reset();
  // nav_goals_active_ 由 publish_goal 重设，不在此无条件清零——防止旧 goal
  // 的结果回调覆盖新 goal 的活跃标志（route 推进时可能连续发 goal）
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      context_.set_nav_status(NavStatus::ARRIVED);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      context_.set_nav_status(NavStatus::IDLE);
      nav_goals_active_ = false;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      context_.set_nav_status(NavStatus::FAILED);
      nav_goals_active_ = false;
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      context_.set_nav_status(NavStatus::FAILED);
      nav_goals_active_ = false;
      break;
  }
}

void DecisionNode::send_stance_command(StanceCommand stance)
{
  rm_interfaces::msg::SentryCommand cmd;
  cmd.stance_command = static_cast<uint8_t>(stance);
  pub_sentry_cmd_->publish(cmd);

  const char * stance_names[] = {
    "NONE",
    "OFFENSIVE",
    "DEFENSIVE",
    "MOBILITY",
    "ENHANCED_OFFENSIVE",
    "ENHANCED_DEFENSIVE",
    "ENHANCED_MOBILITY"};
  const auto idx = static_cast<uint8_t>(stance);
  RCLCPP_DEBUG(
    get_logger(), "SentryCommand published: stance=%s (%u)",
    idx < 7 ? stance_names[idx] : "INVALID", idx);
}



void DecisionNode::auto_aim_target_callback(const std_msgs::msg::String::SharedPtr msg)
{
  bool detected;
  double distance;
  if (!parse_auto_aim_target(msg->data, detected, distance)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "invalid auto aim target payload '%s', expected x,y,valid,id", msg->data.c_str());
    return;
  }
  // 融合模式: 自瞄只管检测+距离, 不碰雷达的坐标字段
  context_.update_enemy_from_aim(detected, distance);
}

bool DecisionNode::parse_auto_aim_target(const std::string & text, bool & detected, double & distance)
{
  std::vector<double> values;
  values.reserve(4);

  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::string token =
      text.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    if (token.empty()) {
      return false;
    }

    try {
      values.push_back(std::stod(token));
    } catch (const std::exception &) {
      return false;
    }
    // 提前拒绝超长恶意输入，防 DoS（期望恰好 4 个字段）
    if (values.size() > 4) return false;

    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }

  if (values.size() != 4) {
    return false;
  }

  const double x = values[0];
  const double y = values[1];
  const double valid = values[2];
  const double id = values[3];

  detected = valid > 0.5 && id > 0.5;
  distance = detected ? std::hypot(x, y) : 999.0;
  return true;
}

void DecisionNode::radar_callback(const radar_msgs::msg::EnemyPosition::SharedPtr msg)
{
  if (msg->robot_type >= radar_positions_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "radar robot_type %u out of range [0,%zu), dropping", msg->robot_type,
      radar_positions_.size());
    return;
  }
  auto & slot = radar_positions_[msg->robot_type];
  slot.x = msg->x;
  slot.y = msg->y;
  slot.valid = (msg->x != 0.0f || msg->y != 0.0f);

  // 融合模式: 聚合雷达数据, 只更新雷达负责的字段(坐标+数量+空中威胁)
  double nearest_x = 0.0, nearest_y = 0.0;
  double min_dist = 999.0;
  int count = 0;
  const bool pos_valid = context_.sentry_pos_valid();
  for (const auto & s : radar_positions_) {
    if (!s.valid) continue;
    if (pos_valid) {
      double dist = std::hypot(s.x - context_.sentry_x(), s.y - context_.sentry_y());
      if (dist < min_dist) {
        min_dist = dist;
        nearest_x = s.x;
        nearest_y = s.y;
      }
    } else if (count == 0) {
      // odom 未就绪，使用第一个有效目标坐标作为最近参考
      nearest_x = s.x;
      nearest_y = s.y;
    }
    count++;
  }
  bool aerial = radar_positions_[4].valid;  // TYPE_AERIAL
  context_.update_enemy_from_radar(nearest_x, nearest_y, count, aerial, min_dist);
}


void DecisionNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  context_.set_sentry_position(current_x_, current_y_);
}

}  // namespace sentry_decision

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_decision::DecisionNode)
