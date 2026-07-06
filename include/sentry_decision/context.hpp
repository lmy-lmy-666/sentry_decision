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
  }
  void update(const rm_interfaces::msg::RfidStatus & m) { rfid_status_ = m; }
  void update(const rm_interfaces::msg::GameRobotHP & m) { robot_hp_ = m; }
  void update(const EnemyInfo & m)
  {
    enemy_info_ = m;
    enemy_info_stamp_ = now();
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

  bool outpost_alive() const { return robot_hp_ && robot_hp_->ally_outpost_hp > 0; }
  uint16_t ally_base_hp() const { return robot_hp_ ? robot_hp_->ally_base_hp : 0; }

  bool on_supply_pad() const
  {
    return rfid_status_ && (rfid_status_->friendly_supply_zone_non_exchange ||
                            rfid_status_->friendly_supply_zone_exchange);
  }

  bool on_base_gain_point() const { return rfid_status_ && rfid_status_->base_gain_point; }

  bool under_attack() const
  {
    return robot_status_ && robot_status_->is_hp_deduced &&
           robot_status_->hp_deduction_reason == rm_interfaces::msg::RobotStatus::ARMOR_HIT;
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

  bool enemy_info_fresh() const { return enemy_info_ && (now() - enemy_info_stamp_) < 0.5; }

  bool under_aerial_attack() const { return enemy_info_fresh() && enemy_info_->aerial_threat; }

  bool enemy_detected() const { return enemy_info_fresh() && enemy_info_->detected; }

  double enemy_distance() const
  {
    return enemy_info_fresh() ? enemy_info_->nearest_distance : 999.0;
  }

  int enemy_count() const { return enemy_info_fresh() ? enemy_info_->count : 0; }

  bool enemy_near_base() const { return enemy_info_fresh() && enemy_info_->near_base; }

  bool enemy_near_outpost() const { return enemy_info_fresh() && enemy_info_->near_outpost; }

  bool double_vulnerability_active() const
  {
    return enemy_info_fresh() && enemy_info_->double_vulnerability_active;
  }

  bool stance_expiring() const { return false; }
  double enhanced_defense_remaining() const
  {
    return sentry_info_ ? sentry_info_->enhanced_defense_remaining_s : 0.0;
  }

  void set_nav_status(NavStatus s) { nav_status_ = s; }
  NavStatus nav_status() const { return nav_status_; }
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
  double enemy_info_stamp_{0.0};
  std::optional<SentryInfo> sentry_info_;

  double game_status_stamp_{0.0};
  double robot_status_stamp_{0.0};
  NavStatus nav_status_{NavStatus::IDLE};
  Thresholds thresholds_;
};

}  // namespace sentry_decision

#endif  // SENTRY_DECISION__CONTEXT_HPP_
