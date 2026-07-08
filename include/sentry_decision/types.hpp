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
#ifndef SENTRY_DECISION__TYPES_HPP_
#define SENTRY_DECISION__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace sentry_decision
{

/// A navigation goal in the map frame. Orientation is intentionally omitted:
/// the sentry aims with the gimbal, so chassis heading is free.
struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double dwell_s{0.0};
};

using Route = std::vector<Waypoint>;

/// Top-level macro states. Keep this set small: safety / logistics / combat /
/// route selection live here; detailed combat choreography is a sub-state.
enum class State {
  IDLE,
  PATROL,
  DEFEND,
  ATTACK_PUSH,
  RESUPPLY,
  RETREAT,
};

inline const char * to_string(State s)
{
  switch (s) {
    case State::IDLE:
      return "IDLE";
    case State::PATROL:
      return "PATROL";
    case State::DEFEND:
      return "DEFEND";
    case State::ATTACK_PUSH:
      return "ATTACK_PUSH";
    case State::RESUPPLY:
      return "RESUPPLY";
    case State::RETREAT:
      return "RETREAT";
  }
  return "UNKNOWN";
}

enum class CombatSubState {
  SCOUT,
  TRACK,
  ENGAGE,
  EVADE,
  HARDEN,
  EVADE_AIR,
};

inline const char * to_string(CombatSubState s)
{
  switch (s) {
    case CombatSubState::SCOUT:
      return "SCOUT";
    case CombatSubState::TRACK:
      return "TRACK";
    case CombatSubState::ENGAGE:
      return "ENGAGE";
    case CombatSubState::EVADE:
      return "EVADE";
    case CombatSubState::HARDEN:
      return "HARDEN";
    case CombatSubState::EVADE_AIR:
      return "EVADE_AIR";
  }
  return "UNKNOWN_COMBAT";
}

enum class NavStatus {
  IDLE,
  MOVING,
  ARRIVED,
  STUCK,
  FAILED,
};

enum class AttackSource {
  NONE,
  GROUND,
  AERIAL,
};

/// Mapped to RM2026 sentry_cmd 0x0120 bit21-23 once the downlink exists.
enum class StanceCommand : uint8_t {
  NONE = 0,
  OFFENSIVE = 1,
  DEFENSIVE = 2,
  MOBILITY = 3,
  ENHANCED_OFFENSIVE = 4,
  ENHANCED_DEFENSIVE = 5,
  ENHANCED_MOBILITY = 6,
};

/// Semantic enemy snapshot. DecisionNode will fill this from auto-aim / radar
/// once those topics are available; tests can inject it directly today.
struct EnemyInfo
{
  double nearest_distance{999.0};
  double nearest_x{0.0};
  double nearest_y{0.0};
  int count{0};
  bool aerial_threat{false};
};

/// Semantic sentry/referee snapshot for 0x020D-like information. Currently a
/// placeholder until the serial protocol exposes it.
struct SentryInfo
{
  bool disengaged{false};
  StanceCommand current_stance{StanceCommand::NONE};
  bool stance_enhanced{false};
  double enhanced_defense_remaining_s{0.0};
};

struct Thresholds
{
  uint16_t hp_low{150};
  uint16_t hp_critical{60};
  uint16_t heat_max{240};
  uint16_t ammo_min{1};
  int32_t game_total_time{420};

  uint8_t min_ticks_in_state{4};
  uint16_t hp_low_exit_hysteresis{180};
  uint16_t hp_critical_exit{120};

  double stuck_timeout_s{30.0};
  double resupply_timeout_s{60.0};
  double retreat_timeout_s{90.0};
  double enemy_lost_time_s{5.0};

  double referee_stale_timeout_s{3.0};
  double enemy_stale_timeout_s{1.0};
  double supply_retry_cooldown_s{15.0};
  double engage_distance{3.0};
  double track_distance{8.0};
  double enhanced_defense_duration_s{15.0};
};

}  // namespace sentry_decision

#endif  // SENTRY_DECISION__TYPES_HPP_
