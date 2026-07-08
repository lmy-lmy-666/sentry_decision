#!/bin/bash
# ============================================================
# omni_navigation 完整录制命令
# 覆盖：导航系统全部话题 + sentry_decision 输入输出
# ============================================================

ros2 bag record \
  `# === 导航核心 ===` \
  /tf \
  /tf_static \
  /map \
  /robot_description \
  \
  `# === 传感器（Livox MID-360） ===` \
  /livox/lidar \
  /livox/imu \
  \
  `# === 定位 (Point-LIO + odom_bridge) ===` \
  /aft_mapped_to_init \
  /cloud_registered \
  /odometry \
  /lidar_odometry \
  \
  `# === 点云处理 ===` \
  /terrain_map \
  /terrain_map_ext \
  /registered_scan \
  \
  `# === Nav2 输出 ===` \
  /plan \
  /local_plan \
  /cmd_vel_chassis \
  /cmd_vel_nav2_result \
  /global_costmap/costmap_raw \
  /local_costmap/costmap_raw \
  \
  `# === 裁判系统 (sentry_decision 核心输入) ===` \
  /referee/game_status \
  /referee/robot_status \
  /referee/rfidStatus \
  /referee/all_robot_hp \
  \
  `# === 感知（外部机器） ===` \
  /auto_aim_target_pos \
  /radar/enemy_positions \
  \
  `# === sentry_decision 输出（调试用） ===` \
  /goal_pose \
  /sentry/command \
  \
  -o omni_decision_full_$(date +%Y%m%d_%H%M%S)
