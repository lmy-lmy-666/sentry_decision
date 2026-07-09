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
  Waypoint supply;           ///< supply pad — also the sole retreat destination

  // --- optional tactical variants (selected at runtime) ----------
  Route patrol_aggressive;   ///< used when outpost is alive (前哨存活激进巡逻)
  Route patrol_late;         ///< used in last 60 s (后期保守巡逻)

  // --- fallback chains -------------------------------------------
  std::vector<Waypoint> backup_supply_points;
};

/// Parse a YAML profile. Throws std::runtime_error on invalid input.
Profile load_profile(const std::string & yaml_path);

}  // namespace sentry_decision_sample

#endif  // SENTRY_DECISION_SAMPLE__PROFILE_HPP_
