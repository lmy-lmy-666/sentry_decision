# Copyright 2026 Boombroke
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Launch the FSM decision node with a selectable tactic profile.
# A profile is a YAML file under config/profiles/<profile>.yaml describing
# waypoints / thresholds. Swap tactics with `profile:=<name>` — no rebuild.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_decision")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace."
    )
    declare_profile = DeclareLaunchArgument(
        "profile",
        default_value="rmuc_blue",
        description="Tactic profile name (file stem under config/profiles/).",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false"
    )
    declare_tick = DeclareLaunchArgument(
        "tick_frequency", default_value="10.0", description="FSM tick rate (Hz)."
    )
    declare_goal_topic = DeclareLaunchArgument(
        "goal_topic", default_value="/goal_pose"
    )
    declare_auto_aim_target_topic = DeclareLaunchArgument(
        "auto_aim_target_topic",
        default_value="auto_aim_target_pos",
        description="Auto-aim target topic in std_msgs/String format: x,y,valid,id.",
    )

    namespace = LaunchConfiguration("namespace")
    profile = LaunchConfiguration("profile")
    use_sim_time = LaunchConfiguration("use_sim_time")
    tick_frequency = LaunchConfiguration("tick_frequency")
    goal_topic = LaunchConfiguration("goal_topic")
    auto_aim_target_topic = LaunchConfiguration("auto_aim_target_topic")

    profile_path = PathJoinSubstitution(
        [FindPackageShare("sentry_decision"), "config", "profiles",
         [profile, ".yaml"]]
    )

    decision_node = Node(
        package="sentry_decision",
        executable="sentry_decision_node",
        name="sentry_decision",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "profile_path": profile_path,
                "use_sim_time": use_sim_time,
                "tick_frequency": tick_frequency,
                "goal_topic": goal_topic,
                "auto_aim_target_topic": auto_aim_target_topic,
                "goal_frame": "map",
            }
        ],
    )

    # Silence unused-import lint for pkg_share (kept for clarity / future use).
    _ = pkg_share

    return LaunchDescription(
        [
            declare_namespace,
            declare_profile,
            declare_use_sim_time,
            declare_tick,
            declare_goal_topic,
            declare_auto_aim_target_topic,
            decision_node,
        ]
    )
