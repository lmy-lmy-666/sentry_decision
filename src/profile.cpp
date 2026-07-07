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
#include "sentry_decision/profile.hpp"

#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace sentry_decision
{
namespace
{

Waypoint parse_waypoint(const YAML::Node & node)
{
  Waypoint wp;
  wp.x = node["x"].as<double>();
  wp.y = node["y"].as<double>();
  if (node["dwell_s"]) {
    wp.dwell_s = node["dwell_s"].as<double>();
  }
  return wp;
}

Route parse_route(const YAML::Node & node)
{
  Route route;
  if (!node || !node.IsSequence()) return route;
  for (const auto & item : node) {
    route.push_back(parse_waypoint(item));
  }
  return route;
}

template <typename T>
T get_or(const YAML::Node & node, const char * key, T fallback)
{
  return (node && node[key]) ? node[key].as<T>() : fallback;
}

}  // namespace

Profile load_profile(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("failed to load decision profile '" + yaml_path + "': " + e.what());
  }

  Profile p;
  p.name = get_or<std::string>(root, "name", "unnamed");

  const Thresholds def;
  const YAML::Node th = root["thresholds"];
  p.thresholds.hp_low = get_or<uint16_t>(th, "hp_low", def.hp_low);
  p.thresholds.hp_critical = get_or<uint16_t>(th, "hp_critical", def.hp_critical);
  p.thresholds.heat_max = get_or<uint16_t>(th, "heat_max", def.heat_max);
  p.thresholds.ammo_min = get_or<uint16_t>(th, "ammo_min", def.ammo_min);
  p.thresholds.game_total_time = get_or<int32_t>(th, "game_total_time", def.game_total_time);
  p.thresholds.min_ticks_in_state =
    get_or<uint8_t>(th, "min_ticks_in_state", def.min_ticks_in_state);
  p.thresholds.hp_low_exit_hysteresis =
    get_or<uint16_t>(th, "hp_low_exit_hysteresis", def.hp_low_exit_hysteresis);
  p.thresholds.hp_critical_exit = get_or<uint16_t>(th, "hp_critical_exit", def.hp_critical_exit);
  p.thresholds.referee_stale_timeout_s =
    get_or<double>(th, "referee_stale_timeout_s", def.referee_stale_timeout_s);
  p.thresholds.stuck_timeout_s = get_or<double>(th, "stuck_timeout_s", def.stuck_timeout_s);
  p.thresholds.resupply_timeout_s =
    get_or<double>(th, "resupply_timeout_s", def.resupply_timeout_s);
  p.thresholds.retreat_timeout_s = get_or<double>(th, "retreat_timeout_s", def.retreat_timeout_s);
  p.thresholds.enemy_lost_time_s = get_or<double>(th, "enemy_lost_time_s", def.enemy_lost_time_s);
  p.thresholds.engage_distance = get_or<double>(th, "engage_distance", def.engage_distance);
  p.thresholds.track_distance = get_or<double>(th, "track_distance", def.track_distance);
  p.thresholds.enhanced_defense_duration_s =
    get_or<double>(th, "enhanced_defense_duration_s", def.enhanced_defense_duration_s);
  p.thresholds.enemy_stale_timeout_s =
    get_or<double>(th, "enemy_stale_timeout_s", def.enemy_stale_timeout_s);
  p.thresholds.supply_retry_cooldown_s =
    get_or<double>(th, "supply_retry_cooldown_s", def.supply_retry_cooldown_s);

  p.patrol = parse_route(root["patrol"]);
  p.attack_push = parse_route(root["attack_push"]);
  p.defend_fallback = parse_route(root["defend_fallback"]);
  if (root["supply"]) p.supply = parse_waypoint(root["supply"]);
  if (root["retreat"]) p.retreat = parse_waypoint(root["retreat"]);
  if (root["safe_cover"]) p.safe_cover = parse_waypoint(root["safe_cover"]);

  p.enable_attack_push = get_or<bool>(root, "enable_attack_push", true);
  if (root["backup_supply_points"])
    p.backup_supply_points = parse_route(root["backup_supply_points"]);
  if (root["backup_retreat_points"])
    p.backup_retreat_points = parse_route(root["backup_retreat_points"]);

  if (root["late_game_leading"]) {
    auto lg = root["late_game_leading"];
    p.late_game_leading.disable_attack_push = get_or<bool>(lg, "disable_attack_push", true);
    p.late_game_leading.fallback_patrol = parse_route(lg["fallback_patrol"]);
  }
  if (root["late_game_trailing"]) {
    auto lg = root["late_game_trailing"];
    p.late_game_trailing.disable_attack_push = get_or<bool>(lg, "disable_attack_push", false);
    p.late_game_trailing.fallback_patrol = parse_route(lg["fallback_patrol"]);
  }

  if (p.patrol.empty() && p.attack_push.empty()) {
    throw std::runtime_error(
      "decision profile '" + yaml_path + "' defines neither 'patrol' nor 'attack_push' waypoints");
  }

  return p;
}

}  // namespace sentry_decision
