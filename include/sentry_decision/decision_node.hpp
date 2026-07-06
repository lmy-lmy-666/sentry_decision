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
#ifndef SENTRY_DECISION__DECISION_NODE_HPP_
#define SENTRY_DECISION__DECISION_NODE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "radar_msgs/msg/enemy_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/motion_state.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "rm_interfaces/msg/sentry_command.hpp"
#include "sentry_decision/context.hpp"
#include "sentry_decision/fsm.hpp"
#include "std_msgs/msg/string.hpp"

namespace sentry_decision
{

class DecisionNode : public rclcpp::Node
{
public:
  explicit DecisionNode(const rclcpp::NodeOptions & options);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void on_tick();
  void publish_goal(const Waypoint & wp);
  void publish_goal_topic_fallback(const Waypoint & wp);
  void cancel_nav();
  void send_stance_command(StanceCommand stance);
  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
  void motion_state_callback(const std_msgs::msg::String::SharedPtr msg);
  void motion_state_structured_callback(const rm_interfaces::msg::MotionState::SharedPtr msg);
  void auto_aim_target_callback(const std_msgs::msg::String::SharedPtr msg);
  void radar_callback(const radar_msgs::msg::EnemyPosition::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  static bool contains_token(const std::string & text, const std::string & token);
  static bool parse_auto_aim_target(const std::string & text, EnemyInfo & enemy);

  Context context_;
  std::unique_ptr<DecisionFsm> fsm_;
  State last_logged_state_{State::IDLE};
  CombatSubState last_logged_combat_substate_{CombatSubState::SCOUT};
  bool nav_goals_active_{false};

  rclcpp::Subscription<rm_interfaces::msg::GameStatus>::SharedPtr sub_game_status_;
  rclcpp::Subscription<rm_interfaces::msg::RobotStatus>::SharedPtr sub_robot_status_;
  rclcpp::Subscription<rm_interfaces::msg::RfidStatus>::SharedPtr sub_rfid_status_;
  rclcpp::Subscription<rm_interfaces::msg::GameRobotHP>::SharedPtr sub_robot_hp_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_motion_state_;
  rclcpp::Subscription<rm_interfaces::msg::MotionState>::SharedPtr sub_motion_state_structured_;
  rclcpp::Subscription<radar_msgs::msg::EnemyPosition>::SharedPtr sub_radar_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_auto_aim_target_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<rm_interfaces::msg::SentryCommand>::SharedPtr pub_sentry_cmd_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;

  std::string goal_frame_{"map"};
  std::string nav_action_name_{"navigate_to_pose"};
  double goal_reached_distance_tolerance_{0.25};
  double current_x_{0.0};
  double current_y_{0.0};
  double fallback_goal_x_{0.0};
  double fallback_goal_y_{0.0};
  bool fallback_goal_active_{false};
  int idle_detection_counter_{0};

  struct RadarSlot
  {
    double x{0.0};
    double y{0.0};
    bool valid{false};
  };
  std::array<RadarSlot, 6> radar_positions_{};
};

}  // namespace sentry_decision

#endif  // SENTRY_DECISION__DECISION_NODE_HPP_
