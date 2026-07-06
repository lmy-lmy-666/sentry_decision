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
  const std::string motion_state_topic =
    this->declare_parameter<std::string>("motion_state_topic", "motion_manager/state");
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
    "referee/rfid_status", 10,
    [this](const rm_interfaces::msg::RfidStatus::SharedPtr m) { context_.update(*m); });
  sub_robot_hp_ = create_subscription<rm_interfaces::msg::GameRobotHP>(
    "referee/all_robot_hp", 10,
    [this](const rm_interfaces::msg::GameRobotHP::SharedPtr m) { context_.update(*m); });
  sub_motion_state_ = create_subscription<std_msgs::msg::String>(
    motion_state_topic, 10,
    std::bind(&DecisionNode::motion_state_callback, this, std::placeholders::_1));
  sub_motion_state_structured_ = create_subscription<rm_interfaces::msg::MotionState>(
    "motion_manager/motion_state", 10,
    std::bind(&DecisionNode::motion_state_structured_callback, this, std::placeholders::_1));
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
    return;
  }
  if (current_goal_handle_) {
    (void)nav_action_client_->async_cancel_goal(current_goal_handle_);
  } else {
    (void)nav_action_client_->async_cancel_all_goals();
  }
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
  if (current_goal_handle_ && goal_handle != current_goal_handle_) {
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
  current_goal_handle_.reset();
  nav_goals_active_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      context_.set_nav_status(NavStatus::ARRIVED);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      context_.set_nav_status(NavStatus::IDLE);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      context_.set_nav_status(NavStatus::FAILED);
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      context_.set_nav_status(NavStatus::FAILED);
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

void DecisionNode::motion_state_structured_callback(
  const rm_interfaces::msg::MotionState::SharedPtr msg)
{
  if (msg->emergency_stop) {
    context_.set_nav_status(NavStatus::FAILED);
    idle_detection_counter_ = 0;
    return;
  }
  if (
    msg->recovery_phase == rm_interfaces::msg::MotionState::PHASE_STRAIGHT_RELEASE ||
    msg->recovery_phase == rm_interfaces::msg::MotionState::PHASE_LOW_CURVATURE_RELEASE ||
    msg->recovery_phase == rm_interfaces::msg::MotionState::PHASE_ARC_ESCAPE) {
    context_.set_nav_status(NavStatus::STUCK);
    idle_detection_counter_ = 0;
    return;
  }
  if (msg->mode == rm_interfaces::msg::MotionState::MODE_NAVIGATION && msg->has_fresh_command) {
    context_.set_nav_status(NavStatus::MOVING);
    idle_detection_counter_ = 0;
    return;
  }
  if (msg->mode == rm_interfaces::msg::MotionState::MODE_IDLE && nav_goals_active_) {
    idle_detection_counter_++;
    if (idle_detection_counter_ >= 3) {
      context_.set_nav_status(NavStatus::FAILED);
    }
  } else {
    idle_detection_counter_ = 0;
  }
}

void DecisionNode::motion_state_callback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & s = msg->data;
  if (contains_token(s, "emergency_stop=true")) {
    context_.set_nav_status(NavStatus::FAILED);
    idle_detection_counter_ = 0;
    return;
  }
  if (
    contains_token(s, "recovery_phase=straight_release") ||
    contains_token(s, "recovery_phase=low_curvature_release") ||
    contains_token(s, "recovery_phase=arc_escape")) {
    context_.set_nav_status(NavStatus::STUCK);
    idle_detection_counter_ = 0;
    return;
  }
  if (contains_token(s, "mode=navigation") && contains_token(s, "has_fresh_command=true")) {
    context_.set_nav_status(NavStatus::MOVING);
    idle_detection_counter_ = 0;
    return;
  }
  if (contains_token(s, "mode=idle") && nav_goals_active_) {
    idle_detection_counter_++;
    if (idle_detection_counter_ >= 3) {
      context_.set_nav_status(NavStatus::FAILED);
    }
  } else {
    idle_detection_counter_ = 0;
  }
}

void DecisionNode::auto_aim_target_callback(const std_msgs::msg::String::SharedPtr msg)
{
  EnemyInfo enemy;
  if (!parse_auto_aim_target(msg->data, enemy)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "invalid auto aim target payload '%s', expected x,y,valid,id", msg->data.c_str());
    return;
  }
  context_.update(enemy);
}

bool DecisionNode::parse_auto_aim_target(const std::string & text, EnemyInfo & enemy)
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

  enemy.detected = valid > 0.5 && id > 0.5;
  enemy.nearest_distance = enemy.detected ? std::hypot(x, y) : 999.0;
  enemy.count = enemy.detected ? 1 : 0;
  return true;
}

void DecisionNode::radar_callback(const radar_msgs::msg::EnemyPosition::SharedPtr msg)
{
  if (msg->robot_type >= radar_positions_.size()) return;
  auto & slot = radar_positions_[msg->robot_type];
  slot.x = msg->x;
  slot.y = msg->y;
  slot.valid = (msg->x != 0.0f || msg->y != 0.0f);

  // Aggregate all valid radar positions into EnemyInfo
  EnemyInfo enemy;
  double min_dist = 999.0;
  int count = 0;
  for (const auto & s : radar_positions_) {
    if (!s.valid) continue;
    double dist = std::hypot(s.x, s.y);
    if (dist < min_dist) min_dist = dist;
    count++;
  }
  if (radar_positions_[4].valid) enemy.aerial_threat = true;  // TYPE_AERIAL
  enemy.detected = count > 0;
  enemy.count = count;
  enemy.nearest_distance = min_dist;
  context_.update(enemy);
}

bool DecisionNode::contains_token(const std::string & text, const std::string & token)
{
  const auto pos = text.find(token);
  if (pos == std::string::npos) return false;

  // 检查左边界：token 必须在字符串开头、空格后或逗号后
  if (pos > 0 && text[pos - 1] != ' ' && text[pos - 1] != ',') return false;

  // 检查右边界：token 必须在字符串结尾、空格前或逗号前
  const auto end = pos + token.size();
  if (end < text.size() && text[end] != ' ' && text[end] != ',') return false;

  return true;
}

void DecisionNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
}

}  // namespace sentry_decision

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_decision::DecisionNode)
