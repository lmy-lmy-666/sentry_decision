#!/usr/bin/env python3
"""
Referee simulator for basic sentry_decision navigation testing.
Publishes 4 referee topics to drive the FSM through IDLE→PATROL→RETREAT→RESUPPLY→PATROL.

Usage:
    ros2 run sentry_decision referee_simulator.py

Then open another terminal for interactive control:
    # Simulate HP drop (triggers RETREAT at hp<60)
    ros2 param set /referee_simulator hp 50

    # Simulate low HP + empty ammo (triggers RESUPPLY at hp<150 or ammo=0)
    ros2 param set /referee_simulator hp 120
    ros2 param set /referee_simulator ammo 0

    # Simulate entering supply zone (resets supply_backup_exhausted)
    ros2 param set /referee_simulator on_supply_pad true
    # Leave supply zone
    ros2 param set /referee_simulator on_supply_pad false

    # Recover HP (exits RETREAT at hp>=120, exits RESUPPLY at hp>=400)
    ros2 param set /referee_simulator hp 400

    # Simulate overheat risk (prevents ATTACK_PUSH)
    ros2 param set /referee_simulator heat 300

    # Simulate late game (t < 60s)
    ros2 param set /referee_simulator remain_time 50

    # Pause game (triggers IDLE)
    ros2 param set /referee_simulator game_running false

Test sequence for basic navigation:
    1. game_running=true, hp=400, ammo=100 → PATROL (sends patrol waypoints)
    2. hp=50                     → RETREAT (sends retreat waypoint)
    3. hp=125, ammo=0            → RESUPPLY (sends supply waypoint)
    4. on_supply_pad=true        → stays, resets exhausted
    5. hp=400, ammo=100          → PATROL
    6. game_running=false        → IDLE (cancels all nav)
"""

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import GameStatus, RobotStatus, RfidStatus, GameRobotHP


class RefereeSimulator(Node):
    def __init__(self):
        super().__init__("referee_simulator")

        # --- Publishers ---
        self.pub_game = self.create_publisher(GameStatus, "referee/game_status", 10)
        self.pub_robot = self.create_publisher(RobotStatus, "referee/robot_status", 10)
        self.pub_rfid = self.create_publisher(RfidStatus, "referee/rfidStatus", 10)
        self.pub_hp = self.create_publisher(GameRobotHP, "referee/all_robot_hp", 10)

        # --- Adjustable params ---
        self.declare_parameter("hp", 400)
        self.declare_parameter("ammo", 100)
        self.declare_parameter("heat", 0)
        self.declare_parameter("on_supply_pad", False)
        self.declare_parameter("remain_time", 300)
        self.declare_parameter("game_running", True)
        self.declare_parameter("rate_hz", 10.0)

        self.timer = self.create_timer(
            1.0 / self.get_parameter("rate_hz").value, self.tick
        )

        self.start_time = self.get_clock().now()
        self.get_logger().info("Referee simulator started. Use 'ros2 param set /referee_simulator <key> <val>'")

    def tick(self):
        hp = self.get_parameter("hp").value
        ammo = self.get_parameter("ammo").value
        heat = self.get_parameter("heat").value
        on_supply_pad = self.get_parameter("on_supply_pad").value
        remain_time = self.get_parameter("remain_time").value
        game_running = self.get_parameter("game_running").value

        # --- GameStatus ---
        gs = GameStatus()
        gs.game_progress = GameStatus.RUNNING if game_running else GameStatus.COUNT_DOWN
        gs.stage_remain_time = int(remain_time)

        # --- RobotStatus ---
        rs = RobotStatus()
        rs.robot_id = 7  # sentry
        rs.robot_level = 1
        rs.current_hp = int(hp)
        rs.maximum_hp = 400
        rs.projectile_allowance_17mm = int(ammo)
        rs.shooter_17mm_1_barrel_heat = int(heat)
        rs.shooter_barrel_cooling_value = 40
        rs.shooter_barrel_heat_limit = 240
        rs.is_hp_deduced = False
        rs.hp_deduction_reason = 0

        # --- RfidStatus ---
        rf = RfidStatus()
        rf.friendly_supply_zone_non_exchange = on_supply_pad
        rf.friendly_supply_zone_exchange = False

        # --- GameRobotHP ---
        hp_all = GameRobotHP()
        hp_all.ally_outpost_hp = 1500
        hp_all.ally_base_hp = 5000

        # --- Publish ---
        self.pub_game.publish(gs)
        self.pub_robot.publish(rs)
        self.pub_rfid.publish(rf)
        self.pub_hp.publish(hp_all)


def main():
    rclpy.init()
    node = RefereeSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
