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
enum class State {
  IDLE,       ///< referee offline or game not running
  PATROL,     ///< default: follow patrol route
  RESUPPLY,   ///< low hp or empty ammo → go to supply pad
  RETREAT,    ///< critical hp → fall back to safety
};

inline const char * to_string(State s)
{
  switch (s) {
    case State::IDLE:     return "IDLE";
    case State::PATROL:   return "PATROL";
    case State::RESUPPLY: return "RESUPPLY";
    case State::RETREAT:  return "RETREAT";
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
  // --- hp / ammo -------------------------------------------------
  uint16_t hp_low{150};
  uint16_t hp_critical{60};
  uint16_t hp_critical_exit{120};   ///< hysteresis: leave RETREAT when hp ≥ this
  uint16_t ammo_min{1};            ///< ≤ this = empty

  // --- timing ----------------------------------------------------
  int32_t game_total_time{420};    ///< match duration (s)
  uint8_t min_ticks_in_state{4};   ///< min ticks before non-safety state exit

  double stuck_timeout_s{30.0};    ///< per-waypoint timeout

  double resupply_timeout_s{30.0};        ///< single supply-point timeout
  double resupply_total_timeout_s{120.0}; ///< cap for entire RESUPPLY chain
  double retreat_timeout_s{30.0};         ///< single retreat-point timeout
  double retreat_total_timeout_s{60.0};   ///< cap for entire RETREAT chain

  double referee_stale_timeout_s{3.0};    ///< referee data expiry
  double supply_retry_cooldown_s{15.0};   ///< cooldown after all supply points fail
};

}  // namespace sentry_decision_sample

#endif  // SENTRY_DECISION_SAMPLE__TYPES_HPP_
