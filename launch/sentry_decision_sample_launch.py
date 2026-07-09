import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("sentry_decision_sample")
    profile = os.path.join(pkg_dir, "config", "profiles", "rmuc_red.yaml")

    return LaunchDescription([
        Node(
            package="sentry_decision_sample",
            executable="sentry_decision_sample_node",
            name="sentry_decision_sample",
            output="screen",
            parameters=[{
                "profile_path": profile,
                "tick_frequency": 10.0,
                "goal_topic": "/goal_pose",
                "nav_action_name": "navigate_to_pose",
                "goal_frame": "map",
                "goal_reached_distance_tolerance": 0.25,
            }],
        ),
    ])
