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
#ifndef SENTRY_DECISION__PROFILE_HPP_
#define SENTRY_DECISION__PROFILE_HPP_

#include <string>
#include <vector>

#include "sentry_decision/types.hpp"

namespace sentry_decision
{

struct Profile
{
  std::string name;
  Thresholds thresholds;

  Route patrol;
  Route attack_push;
  Route defend_fallback;
  Waypoint supply;
  Waypoint retreat;
  Waypoint safe_cover;

  bool enable_attack_push{true};
  std::vector<Waypoint> backup_supply_points;
  std::vector<Waypoint> backup_retreat_points;

  struct LateGame
  {
    bool disable_attack_push{true};
    Route fallback_patrol;
  };
  LateGame late_game_leading;
  LateGame late_game_trailing;
};

Profile load_profile(const std::string & yaml_path);

}  // namespace sentry_decision

#endif  // SENTRY_DECISION__PROFILE_HPP_
