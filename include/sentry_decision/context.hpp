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
#ifndef SENTRY_DECISION__CONTEXT_HPP_
#define SENTRY_DECISION__CONTEXT_HPP_

#include <cmath>
#include <optional>

#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "sentry_decision/types.hpp"

namespace sentry_decision
{

/// Sensor-fused world model consumed by DecisionFsm::tick().
/// Thread safety: all reads/writes must be serialized externally — this class
/// contains no internal locking and assumes a SingleThreadedExecutor or
/// equivalent guarantee that callbacks and the tick timer share one thread.
class Context
{
public:
  Context() = default;

  void set_thresholds(const Thresholds & t) { thresholds_ = t; }
  const Thresholds & thresholds() const { return thresholds_; }

  void update(const rm_interfaces::msg::GameStatus & m)
  {
    game_status_ = m;
    game_status_stamp_ = now();
  }
  void update(const rm_interfaces::msg::RobotStatus & m)
  {
    robot_status_ = m;
    robot_status_stamp_ = now();
    // 缓存受击时刻：裁判可能在下一帧清除 is_hp_deduced，
    // 但 under_attack() 需要短窗口持久化防止漏检。
    if (m.is_hp_deduced &&
        (m.hp_deduction_reason == rm_interfaces::msg::RobotStatus::ARMOR_HIT ||
         m.hp_deduction_reason == rm_interfaces::msg::RobotStatus::ARMOR_COLLISION)) {
      last_under_attack_s_ = now();
    }
  }
  void update(const rm_interfaces::msg::RfidStatus & m)
  {
    rfid_status_ = m;
    rfid_stamp_ = now();
  }
  void update(const rm_interfaces::msg::GameRobotHP & m)
  {
    robot_hp_ = m;
    robot_hp_stamp_ = now();
  }
  // === 双源敌方信息融合（读时融合） ===
  // 每个源独立维护自己的检测标记，互不覆盖。
  // enemy_detected() 在读取时做 OR 融合——任意一方看到敌人即为真。
  void update_enemy_from_aim(bool detected, double distance)
  {
    if (!enemy_info_) enemy_info_ = EnemyInfo{};
    aim_detected_ = detected;
    enemy_info_->nearest_distance = detected ? distance : 999.0;
    aim_stamp_ = now();
  }
  void update_enemy_from_radar(double x, double y, int count, bool aerial, double min_dist = 999.0)
  {
    if (!enemy_info_) enemy_info_ = EnemyInfo{};
    radar_has_target_ = (count > 0);
    enemy_info_->nearest_x = x;
    enemy_info_->nearest_y = y;
    enemy_info_->count = count;
    enemy_info_->aerial_threat = aerial;
    // 雷达测距（仅在己方 odom 就绪时有效，否则保持自瞄值或默认值）
    if (min_dist < 999.0) enemy_info_->nearest_distance = min_dist;
    radar_stamp_ = now();
  }

  void update(const SentryInfo & m) { sentry_info_ = m; }

  bool has_referee() const { return game_status_ && robot_status_; }
  bool game_status_fresh() const
  {
    return game_status_ && (now() - game_status_stamp_) < thresholds_.referee_stale_timeout_s;
  }
  bool robot_status_fresh() const
  {
    return robot_status_ && (now() - robot_status_stamp_) < thresholds_.referee_stale_timeout_s;
  }
  bool referee_fresh() const { return game_status_fresh() && robot_status_fresh(); }

  bool game_running() const
  {
    if (!game_status_) return false;
    const auto & g = *game_status_;
    return g.game_progress == rm_interfaces::msg::GameStatus::RUNNING && g.stage_remain_time >= 0 &&
           g.stage_remain_time <= thresholds_.game_total_time;
  }

  int32_t remain_time() const { return game_status_ ? game_status_->stage_remain_time : -1; }
  bool late_game() const { return remain_time() >= 0 && remain_time() < 60; }

  uint16_t hp() const { return robot_status_ ? robot_status_->current_hp : 0; }
  uint16_t max_hp() const { return robot_status_ ? robot_status_->maximum_hp : 400; }
  bool hp_critical() const
  {
    return robot_status_ && robot_status_->current_hp < thresholds_.hp_critical;
  }
  bool hp_low() const { return robot_status_ && robot_status_->current_hp < thresholds_.hp_low; }

  uint16_t ammo() const { return robot_status_ ? robot_status_->projectile_allowance_17mm : 0; }
  bool ammo_empty() const
  {
    return robot_status_ && robot_status_->projectile_allowance_17mm <= thresholds_.ammo_min;
  }

  uint16_t barrel_heat() const
  {
    return robot_status_ ? robot_status_->shooter_17mm_1_barrel_heat : 0;
  }
  bool overheat_risk() const
  {
    return robot_status_ && robot_status_->shooter_17mm_1_barrel_heat > thresholds_.heat_max;
  }

  bool needs_resupply() const { return ammo_empty() || hp_low(); }

  bool disengage() const
  {
    if (sentry_info_) return sentry_info_->disengaged;
    return !under_attack();
  }

  bool hp_data_fresh() const
  {
    return robot_hp_ && (now() - robot_hp_stamp_) < thresholds_.referee_stale_timeout_s;
  }
  bool rfid_fresh() const
  {
    return rfid_status_ && (now() - rfid_stamp_) < thresholds_.referee_stale_timeout_s;
  }

  bool outpost_alive() const { return hp_data_fresh() && robot_hp_->ally_outpost_hp > 0; }
  uint16_t ally_base_hp() const { return hp_data_fresh() ? robot_hp_->ally_base_hp : 0; }

  bool on_supply_pad() const
  {
    return rfid_fresh() && (rfid_status_->friendly_supply_zone_non_exchange ||
                            rfid_status_->friendly_supply_zone_exchange);
  }

  bool on_base_gain_point() const { return rfid_fresh() && rfid_status_->base_gain_point; }

  // 装甲中弹 或 被撞击——都是敌方造成的伤害。
  // 裁判可能在上报后清除标志，因此额外维持 0.5s 短窗口防止 tick 错开漏检。
  bool under_attack() const
  {
    if (robot_status_ && robot_status_->is_hp_deduced &&
        (robot_status_->hp_deduction_reason == rm_interfaces::msg::RobotStatus::ARMOR_HIT ||
         robot_status_->hp_deduction_reason == rm_interfaces::msg::RobotStatus::ARMOR_COLLISION)) {
      return true;
    }
    return (now() - last_under_attack_s_) < 0.5;
  }

  uint8_t hit_armor_id() const { return robot_status_ ? robot_status_->armor_id : 0; }

  double attack_direction() const
  {
    switch (hit_armor_id()) {
      case 0:
        return 0.0;
      case 1:
        return std::acos(-1.0) / 2.0;
      case 2:
        return std::acos(-1.0);
      case 3:
        return -std::acos(-1.0) / 2.0;
      default:
        return 0.0;
    }
  }

  // 自瞄或雷达任一有新鲜数据即可，不限来源
  bool enemy_info_fresh() const
  {
    if (!enemy_info_) return false;
    const double t = now();
    return (t - aim_stamp_) < thresholds_.enemy_stale_timeout_s ||
           (t - radar_stamp_) < thresholds_.enemy_stale_timeout_s;
  }

  bool under_aerial_attack() const { return enemy_info_fresh() && enemy_info_->aerial_threat; }

  // 读时融合: 自瞄或雷达任一确认有敌人 → true。互不覆盖，无竞态。
  bool enemy_detected() const
  {
    const double t = now();
    return (aim_detected_ && (t - aim_stamp_) < thresholds_.enemy_stale_timeout_s) ||
           (radar_has_target_ && (t - radar_stamp_) < thresholds_.enemy_stale_timeout_s);
  }

  double enemy_distance() const
  {
    return enemy_info_fresh() ? enemy_info_->nearest_distance : 999.0;
  }

  double nearest_enemy_x() const { return enemy_info_fresh() ? enemy_info_->nearest_x : 0.0; }

  double nearest_enemy_y() const { return enemy_info_fresh() ? enemy_info_->nearest_y : 0.0; }

  int enemy_count() const { return enemy_info_fresh() ? enemy_info_->count : 0; }

  bool stance_expiring() const { return false; }
  double enhanced_defense_remaining() const
  {
    return sentry_info_ ? sentry_info_->enhanced_defense_remaining_s : 0.0;
  }

  void set_nav_status(NavStatus s) { nav_status_ = s; }
  NavStatus nav_status() const { return nav_status_; }

  void set_sentry_position(double x, double y)
  {
    sentry_x_ = x;
    sentry_y_ = y;
    sentry_pos_valid_ = true;
  }
  double sentry_x() const { return sentry_x_; }
  double sentry_y() const { return sentry_y_; }
  bool sentry_pos_valid() const { return sentry_pos_valid_; }
  bool nav_stuck() const
  {
    return nav_status_ == NavStatus::STUCK || nav_status_ == NavStatus::FAILED;
  }
  bool goal_reached() const { return nav_status_ == NavStatus::ARRIVED; }

  uint16_t gold() const { return robot_status_ ? robot_status_->remaining_gold_coin : 0; }

  void set_now(double t) { simulated_now_ = t; }

private:
  double now() const { return simulated_now_; }

  double simulated_now_{0.0};
  std::optional<rm_interfaces::msg::GameStatus> game_status_;
  std::optional<rm_interfaces::msg::RobotStatus> robot_status_;
  std::optional<rm_interfaces::msg::RfidStatus> rfid_status_;
  std::optional<rm_interfaces::msg::GameRobotHP> robot_hp_;
  std::optional<EnemyInfo> enemy_info_;
  double aim_stamp_{0.0};         // 自瞄最后更新时间
  double radar_stamp_{0.0};       // 雷达最后更新时间
  bool aim_detected_{false};      // 自瞄独立检测标记
  bool radar_has_target_{false};  // 雷达独立检测标记
  std::optional<SentryInfo> sentry_info_;

  double game_status_stamp_{0.0};
  double robot_status_stamp_{0.0};
  double rfid_stamp_{0.0};
  double robot_hp_stamp_{0.0};
  mutable double last_under_attack_s_{-1.0};  // 受击持久化: under_attack() 的 0.5s 滞后窗
  NavStatus nav_status_{NavStatus::IDLE};
  double sentry_x_{0.0};
  double sentry_y_{0.0};
  bool sentry_pos_valid_{false};
  Thresholds thresholds_;
};

}  // namespace sentry_decision

#endif  // SENTRY_DECISION__CONTEXT_HPP_
