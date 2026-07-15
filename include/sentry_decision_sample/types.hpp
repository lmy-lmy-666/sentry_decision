// Copyright 2026 Boombroke
//
// Simplified sentry decision types — navigation-only, no combat.
//
#ifndef SENTRY_DECISION_SAMPLE__TYPES_HPP_
#define SENTRY_DECISION_SAMPLE__TYPES_HPP_

#include <cstdint>
#include <vector>

namespace sentry_decision_sample
{

/// A navigation waypoint in map frame. Orientation is omitted:
/// the gimbal auto-aims independently, chassis heading is free.
struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double dwell_s{0.0};  ///< seconds to wait after arrival
};

using Route = std::vector<Waypoint>;

/// Top-level states — keep it minimal.
/// Supply pad doubles as the safe fallback: low hp OR low ammo both send the
/// sentry home, where it heals passively and refills ammo (+100/min, no active
/// exchange). No separate RETREAT — at supply-only granularity a "critical hp"
/// state produces the same navigation action (drive to the pad).
enum class State {
  IDLE,            ///< referee offline or game not running
  OPENING_STRIKE,  ///< match start: drive to a firing spot, dwell to let auto-aim kill enemy outpost (once)
  PATROL,          ///< default: follow patrol route
  RESUPPLY,        ///< low hp or low ammo → go to supply pad, recover, then leave
};

inline const char * to_string(State s)
{
  switch (s) {
    case State::IDLE:            return "IDLE";
    case State::OPENING_STRIKE:  return "OPENING_STRIKE";
    case State::PATROL:          return "PATROL";
    case State::RESUPPLY:        return "RESUPPLY";
  }
  return "UNKNOWN";
}

enum class NavStatus {
  IDLE,
  MOVING,
  ARRIVED,
  FAILED,
};

/// All tunable thresholds. YAML profiles can override the defaults.
struct Thresholds
{
  // --- hp / ammo (hysteresis) ------------------------------------
  // Enter RESUPPLY when hp < hp_low OR ammo <= ammo_low.
  // Leave RESUPPLY only when hp >= max_hp AND ammo >= ammo_ok.
  uint16_t max_hp{400};            ///< full-hp target (auto sentry = 400; serial 0x0201 does NOT report maximum_hp, so configure it here)
  uint16_t hp_low{150};            ///< enter RESUPPLY below this hp
  uint16_t ammo_low{50};           ///< enter RESUPPLY at/below this ammo
  uint16_t ammo_ok{100};           ///< leave RESUPPLY at/above this ammo (one free +100 cycle)

  // --- timing ----------------------------------------------------
  int32_t game_total_time{420};    ///< match duration (s)
  uint8_t min_ticks_in_state{4};   ///< min ticks before state exit (oscillation guard)

  double stuck_timeout_s{10.0};    ///< per-waypoint patrol timeout

  double resupply_timeout_s{30.0}; ///< single supply-point timeout → rotate to next point (never gives up)

  double referee_stale_timeout_s{3.0};    ///< referee data expiry

  // --- opening strike (kill enemy outpost at match start, once) ---
  double opening_strike_duration_s{90.0}; ///< dwell at firing spot to let auto-aim destroy enemy outpost (1.5 min)
};

}  // namespace sentry_decision_sample

#endif  // SENTRY_DECISION_SAMPLE__TYPES_HPP_
