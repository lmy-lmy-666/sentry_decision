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
#ifndef SENTRY_DECISION__FSM_HPP_
#define SENTRY_DECISION__FSM_HPP_

#include <cstddef>
#include <functional>
#include <utility>

#include "sentry_decision/context.hpp"
#include "sentry_decision/profile.hpp"
#include "sentry_decision/types.hpp"

namespace sentry_decision
{

using GoalPublisher = std::function<void(const Waypoint &)>;
using StanceSender = std::function<void(StanceCommand)>;
using NavCanceller = std::function<void()>;

class DecisionFsm
{
public:
  DecisionFsm(
    Profile profile, GoalPublisher goal_pub, StanceSender stance_send = nullptr,
    NavCanceller nav_cancel = nullptr);

  void tick(const Context & ctx, double now_s);

  State state() const { return state_; }
  CombatSubState combat_substate() const { return combat_substate_; }
  const Profile & profile() const { return profile_; }

private:
  State select_state(const Context & ctx) const;
  bool can_leave_current_state(State next) const;
  bool attack_push_allowed(const Context & ctx) const;
  bool can_bypass_stance_cooldown(StanceCommand stance) const;

  void on_enter(State s, double now_s);
  void on_exit(State s);
  void run_behaviour(const Context & ctx, double now_s);

  void behave_idle(const Context & ctx, double now_s);
  void behave_patrol(const Context & ctx, double now_s);
  void behave_defend(const Context & ctx, double now_s);
  void behave_attack_push(const Context & ctx, double now_s);
  void behave_resupply(const Context & ctx, double now_s);
  void behave_retreat(const Context & ctx, double now_s);

  void run_combat_fsm(const Context & ctx, double now_s);
  void combat_scout(const Context & ctx, double now_s);
  void combat_track(const Context & ctx, double now_s);
  void combat_engage(const Context & ctx, double now_s);
  void combat_evade(const Context & ctx, double now_s);
  void combat_harden(const Context & ctx, double now_s);
  void combat_evade_air(const Context & ctx, double now_s);

  void drive_route(const Route & route, double now_s);
  void publish_single_goal(const Waypoint & wp);
  void switch_stance(StanceCommand stance, double now_s, bool force = false);

  Profile profile_;
  GoalPublisher publish_goal_;
  StanceSender send_stance_;
  NavCanceller cancel_nav_;

  State state_{State::IDLE};
  State prev_state_{State::IDLE};
  CombatSubState combat_substate_{CombatSubState::SCOUT};
  AttackSource attack_source_{AttackSource::NONE};

  std::size_t path_idx_{0};
  bool goal_sent_{false};
  double waypoint_started_s_{0.0};
  double waypoint_arrived_s_{0.0};
  bool goal_arrived_{false};
  double state_entered_s_{0.0};
  int ticks_in_state_{0};

  double enemy_lost_at_s_{0.0};
  double last_hit_at_s_{0.0};
  double substate_entered_s_{0.0};
  double operation_started_s_{0.0};
  bool supply_backup_exhausted_{false};

  StanceCommand last_commanded_stance_{StanceCommand::NONE};
  double last_stance_command_s_{-1e9};
};

}  // namespace sentry_decision

#endif  // SENTRY_DECISION__FSM_HPP_
