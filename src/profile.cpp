// Copyright 2026 Boombroke
//
// Simplified YAML profile loader.
//
#include "sentry_decision_sample/profile.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace sentry_decision_sample
{
namespace
{

Waypoint parse_waypoint(const YAML::Node & node)
{
  Waypoint wp;
  wp.x = node["x"].as<double>();
  wp.y = node["y"].as<double>();
  if (node["dwell_s"]) wp.dwell_s = node["dwell_s"].as<double>();

  if (!std::isfinite(wp.x) || !std::isfinite(wp.y) || !std::isfinite(wp.dwell_s)) {
    throw std::runtime_error(
      "waypoint contains NaN/Inf: (" + std::to_string(wp.x) + ", " +
      std::to_string(wp.y) + ", " + std::to_string(wp.dwell_s) + ")");
  }
  return wp;
}

Route parse_route(const YAML::Node & node)
{
  Route route;
  if (!node || !node.IsSequence()) return route;
  for (const auto & item : node) route.push_back(parse_waypoint(item));
  return route;
}

template <typename T>
T get_or(const YAML::Node & node, const char * key, T fallback)
{
  return (node && node[key]) ? node[key].template as<T>() : fallback;
}

}  // namespace

Profile load_profile(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("failed to load profile '" + yaml_path + "': " + e.what());
  }

  Profile p;
  p.name = get_or<std::string>(root, "name", "unnamed");

  // --- thresholds (YAML overrides defaults) -------------------------
  const Thresholds def;
  const YAML::Node th = root["thresholds"];
  p.thresholds.hp_low           = get_or<uint16_t>(th, "hp_low", def.hp_low);
  p.thresholds.ammo_low         = get_or<uint16_t>(th, "ammo_low", def.ammo_low);
  p.thresholds.ammo_ok          = get_or<uint16_t>(th, "ammo_ok", def.ammo_ok);
  p.thresholds.game_total_time  = get_or<int32_t>(th, "game_total_time", def.game_total_time);
  p.thresholds.min_ticks_in_state = get_or<uint8_t>(th, "min_ticks_in_state", def.min_ticks_in_state);
  p.thresholds.stuck_timeout_s     = get_or<double>(th, "stuck_timeout_s", def.stuck_timeout_s);
  p.thresholds.resupply_timeout_s  = get_or<double>(th, "resupply_timeout_s", def.resupply_timeout_s);
  p.thresholds.referee_stale_timeout_s = get_or<double>(th, "referee_stale_timeout_s", def.referee_stale_timeout_s);

  // --- routes & waypoints -------------------------------------------
  p.patrol          = parse_route(root["patrol"]);
  p.patrol_aggressive = parse_route(root["patrol_aggressive"]);
  if (root["supply"]) p.supply = parse_waypoint(root["supply"]);

  if (root["backup_supply_points"])
    p.backup_supply_points = parse_route(root["backup_supply_points"]);

  // --- validation --------------------------------------------------
  if (p.patrol.empty()) {
    throw std::runtime_error("profile '" + yaml_path + "': 'patrol' route is required");
  }

  return p;
}

}  // namespace sentry_decision_sample
