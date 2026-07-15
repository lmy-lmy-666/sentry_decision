// Copyright 2026 Boombroke
//
// YAML profile loader for simplified sentry decision.
//
#ifndef SENTRY_DECISION_SAMPLE__PROFILE_HPP_
#define SENTRY_DECISION_SAMPLE__PROFILE_HPP_

#include <string>
#include <vector>

#include "sentry_decision_sample/types.hpp"

namespace sentry_decision_sample
{

struct Profile
{
  std::string name;
  Thresholds thresholds;

  // --- mandatory waypoints ---------------------------------------
  Route  patrol;             ///< default patrol loop
  Waypoint supply;           ///< supply pad — the sole recovery destination (heal + refill)

  // --- opening strike (optional) ---------------------------------
  Waypoint opening_strike;   ///< match-start firing spot to hit enemy outpost
  bool     has_opening_strike{false};  ///< true if 'opening_strike' is configured

  // --- optional tactical variant (selected at runtime) ----------
  Route patrol_aggressive;   ///< used while our outpost is alive (前哨存活激进前压)
                             ///< when outpost is destroyed → fall back to `patrol` (我方半场防守)

  // --- fallback chains -------------------------------------------
  std::vector<Waypoint> backup_supply_points;
};

/// Parse a YAML profile. Throws std::runtime_error on invalid input.
Profile load_profile(const std::string & yaml_path);

}  // namespace sentry_decision_sample

#endif  // SENTRY_DECISION_SAMPLE__PROFILE_HPP_
