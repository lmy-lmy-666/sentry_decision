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
#include "sentry_decision/fsm.hpp"

#include <utility>

namespace sentry_decision
{

DecisionFsm::DecisionFsm(
  Profile profile, GoalPublisher goal_pub, StanceSender stance_send, NavCanceller nav_cancel)
: profile_(std::move(profile)),
  publish_goal_(std::move(goal_pub)),
  send_stance_(std::move(stance_send)),
  cancel_nav_(std::move(nav_cancel))
{
}

void DecisionFsm::tick(const Context & ctx, double now_s)
{
  const State candidate = select_state(ctx);
  const State next = can_leave_current_state(candidate) ? candidate : state_;

  if (next != state_) {
    on_exit(state_);
    state_ = next;
    on_enter(next, now_s);
  }

  ++ticks_in_state_;
  run_behaviour(ctx, now_s);
}

State DecisionFsm::select_state(const Context & ctx) const
{
  if (!ctx.referee_fresh() || !ctx.game_running()) {
    return State::IDLE;
  }

  if (state_ == State::RETREAT) {
    if (ctx.hp() < profile_.thresholds.hp_critical_exit) return State::RETREAT;
  } else if (ctx.hp_critical()) {
    return State::RETREAT;
  }

  if (ctx.under_aerial_attack()) {
    return State::DEFEND;
  }

  if (state_ == State::RESUPPLY) {
    // 敌人出现时让路给 DEFEND（guard chain 优先级 ⑤）
    if (!ctx.enemy_detected()) {
      if (supply_backup_exhausted_) {
        if (ctx.hp() < profile_.thresholds.hp_critical_exit) {
          return State::RESUPPLY;
        }
      } else if (ctx.ammo_empty() || ctx.hp() < profile_.thresholds.hp_low_exit_hysteresis) {
        return State::RESUPPLY;
      }
    }
  } else if (ctx.needs_resupply() && !ctx.enemy_detected()) {
    return State::RESUPPLY;
  }

  if (ctx.enemy_detected() || ctx.under_attack()) {
    return State::DEFEND;
  }

  if (attack_push_allowed(ctx)) {
    return State::ATTACK_PUSH;
  }

  return State::PATROL;
}

bool DecisionFsm::can_leave_current_state(State next) const
{
  if (next == state_) return true;
  if (state_ == State::IDLE) return true;
  if (next == State::IDLE || next == State::RETREAT || next == State::DEFEND) return true;
  if (next == State::RESUPPLY && state_ != State::RETREAT) return true;
  return ticks_in_state_ >= profile_.thresholds.min_ticks_in_state;
}

bool DecisionFsm::attack_push_allowed(const Context & ctx) const
{
  if (!profile_.enable_attack_push || profile_.attack_push.empty()) return false;
  if (!ctx.outpost_alive()) return false;
  if (ctx.overheat_risk()) return false;
  if (ctx.ammo_empty()) return false;
  if (ctx.hp() < profile_.thresholds.hp_low_exit_hysteresis) return false;
  if (ctx.under_attack()) return false;

  // 比赛后期默认使用保守策略（late_game_leading），暂不支持自动检测领先/落后
  if (ctx.late_game() && profile_.late_game_leading.disable_attack_push) return false;

  return true;
}

void DecisionFsm::on_enter(State s, double now_s)
{
  path_idx_ = 0;
  goal_sent_ = false;
  goal_arrived_ = false;
  waypoint_started_s_ = now_s;
  waypoint_arrived_s_ = 0.0;
  state_entered_s_ = now_s;
  ticks_in_state_ = 0;

  switch (s) {
    case State::IDLE:
      if (cancel_nav_) cancel_nav_();
      break;
    case State::PATROL:
      switch_stance(StanceCommand::MOBILITY, now_s);
      break;
    case State::DEFEND:
      combat_substate_ = CombatSubState::SCOUT;
      substate_entered_s_ = now_s;
      enemy_lost_at_s_ = 0.0;
      last_hit_at_s_ = 0.0;
      break;
    case State::ATTACK_PUSH:
      switch_stance(StanceCommand::OFFENSIVE, now_s);
      break;
    case State::RESUPPLY:
      switch_stance(StanceCommand::MOBILITY, now_s);
      operation_started_s_ = now_s;
      supply_backup_exhausted_ = false;
      break;
    case State::RETREAT:
      switch_stance(StanceCommand::DEFENSIVE, now_s);
      operation_started_s_ = now_s;
      break;
  }
}

void DecisionFsm::on_exit(State s)
{
  prev_state_ = s;
  if (s == State::DEFEND) {
    combat_substate_ = CombatSubState::SCOUT;
    attack_source_ = AttackSource::NONE;
  }
}

void DecisionFsm::run_behaviour(const Context & ctx, double now_s)
{
  if (goal_sent_ && !goal_arrived_ && ctx.goal_reached()) {
    goal_arrived_ = true;
    waypoint_arrived_s_ = now_s;
  }

  switch (state_) {
    case State::IDLE:
      behave_idle(ctx, now_s);
      break;
    case State::PATROL:
      behave_patrol(ctx, now_s);
      break;
    case State::DEFEND:
      behave_defend(ctx, now_s);
      break;
    case State::ATTACK_PUSH:
      behave_attack_push(ctx, now_s);
      break;
    case State::RESUPPLY:
      behave_resupply(ctx, now_s);
      break;
    case State::RETREAT:
      behave_retreat(ctx, now_s);
      break;
  }
}

void DecisionFsm::behave_idle(const Context &, double) {}

void DecisionFsm::behave_patrol(const Context & ctx, double now_s)
{
  const auto & route = (ctx.late_game() && !profile_.late_game_leading.fallback_patrol.empty())
                         ? profile_.late_game_leading.fallback_patrol
                         : profile_.patrol;

  if (ctx.nav_stuck()) {
    if (!route.empty()) path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    return;
  }
  drive_route(route, now_s);
}

void DecisionFsm::behave_defend(const Context & ctx, double now_s)
{
  if (ctx.under_aerial_attack()) {
    attack_source_ = AttackSource::AERIAL;
    if (
      combat_substate_ != CombatSubState::HARDEN && combat_substate_ != CombatSubState::EVADE_AIR) {
      combat_substate_ = CombatSubState::HARDEN;
      goal_sent_ = false;
    }
  } else if (ctx.under_attack() || ctx.enemy_detected()) {
    attack_source_ = AttackSource::GROUND;
    if (
      combat_substate_ == CombatSubState::HARDEN || combat_substate_ == CombatSubState::EVADE_AIR) {
      combat_substate_ = CombatSubState::SCOUT;
      goal_sent_ = false;
    }
  }
  run_combat_fsm(ctx, now_s);
}

void DecisionFsm::run_combat_fsm(const Context & ctx, double now_s)
{
  const auto prev_sub = combat_substate_;

  if (ctx.under_attack()) last_hit_at_s_ = now_s;

  if (!ctx.enemy_detected() && !ctx.under_attack() && !ctx.under_aerial_attack()) {
    if (enemy_lost_at_s_ == 0.0) enemy_lost_at_s_ = now_s;
  } else {
    enemy_lost_at_s_ = 0.0;
  }

  switch (combat_substate_) {
    case CombatSubState::SCOUT:
      combat_scout(ctx, now_s);
      break;
    case CombatSubState::TRACK:
      combat_track(ctx, now_s);
      break;
    case CombatSubState::ENGAGE:
      combat_engage(ctx, now_s);
      break;
    case CombatSubState::EVADE:
      combat_evade(ctx, now_s);
      break;
    case CombatSubState::HARDEN:
      combat_harden(ctx, now_s);
      break;
    case CombatSubState::EVADE_AIR:
      combat_evade_air(ctx, now_s);
      break;
  }

  if (combat_substate_ != prev_sub) {
    substate_entered_s_ = now_s;
  }
}

void DecisionFsm::combat_scout(const Context & ctx, double now_s)
{
  if (ctx.enemy_detected()) {
    combat_substate_ = CombatSubState::TRACK;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }

  if (ctx.under_attack()) {
    combat_substate_ = CombatSubState::EVADE;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }

  if (enemy_lost_at_s_ > 0.0 && now_s - enemy_lost_at_s_ > profile_.thresholds.enemy_lost_time_s) {
    combat_substate_ = CombatSubState::SCOUT;
  }

  // 没有发现敌人时用 defend_fallback 做防守巡逻
  if (!profile_.defend_fallback.empty()) {
    if (ctx.nav_stuck()) {
      path_idx_ = (path_idx_ + 1) % profile_.defend_fallback.size();
      goal_sent_ = false;
    }
    drive_route(profile_.defend_fallback, now_s);
  }
}

void DecisionFsm::combat_track(const Context & ctx, double now_s)
{
  if (!ctx.enemy_detected()) {
    combat_substate_ = CombatSubState::SCOUT;
    return;
  }
  if (ctx.enemy_distance() < profile_.thresholds.engage_distance) {
    combat_substate_ = CombatSubState::ENGAGE;
    switch_stance(StanceCommand::OFFENSIVE, now_s);
    return;
  }
  if (ctx.under_attack() && now_s - last_hit_at_s_ < 0.5) {
    combat_substate_ = CombatSubState::EVADE;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }
  // TRACK 超时保护：20s 未进入 ENGAGE → 退回 SCOUT
  if (now_s - substate_entered_s_ > 20.0) {
    combat_substate_ = CombatSubState::SCOUT;
  }
}

void DecisionFsm::combat_engage(const Context & ctx, double now_s)
{
  if (!ctx.enemy_detected()) {
    combat_substate_ = CombatSubState::SCOUT;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    return;
  }
  if (ctx.enemy_distance() > profile_.thresholds.track_distance) {
    combat_substate_ = CombatSubState::TRACK;
    return;
  }
  if (ctx.under_attack() && now_s - last_hit_at_s_ < 0.5) {
    combat_substate_ = CombatSubState::EVADE;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }
  if (ctx.overheat_risk()) switch_stance(StanceCommand::DEFENSIVE, now_s);

  // ENGAGE 超时保护：30s 后退回 SCOUT
  if (now_s - substate_entered_s_ > 30.0) {
    combat_substate_ = CombatSubState::SCOUT;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
  }
}

void DecisionFsm::combat_evade(const Context & ctx, double now_s)
{
  if (!ctx.under_attack() && now_s - last_hit_at_s_ > 2.0) {
    combat_substate_ = CombatSubState::SCOUT;
    return;
  }
  publish_single_goal(profile_.safe_cover);
}

void DecisionFsm::combat_harden(const Context & ctx, double now_s)
{
  if (
    ctx.enhanced_defense_remaining() > 0.0 ||
    now_s - substate_entered_s_ <= profile_.thresholds.enhanced_defense_duration_s) {
    switch_stance(StanceCommand::ENHANCED_DEFENSIVE, now_s);
  }
  if (!goal_sent_) publish_single_goal(profile_.safe_cover);

  if (now_s - substate_entered_s_ > profile_.thresholds.enhanced_defense_duration_s) {
    combat_substate_ = CombatSubState::EVADE_AIR;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    return;
  }
  if (!ctx.under_aerial_attack()) combat_substate_ = CombatSubState::SCOUT;
}

void DecisionFsm::combat_evade_air(const Context & ctx, double now_s)
{
  switch_stance(StanceCommand::DEFENSIVE, now_s);
  if (!goal_sent_ || ctx.nav_stuck()) publish_single_goal(profile_.safe_cover);
  if (!ctx.under_aerial_attack()) combat_substate_ = CombatSubState::SCOUT;
}

void DecisionFsm::behave_attack_push(const Context & ctx, double now_s)
{
  if (ctx.nav_stuck()) {
    if (!profile_.attack_push.empty()) path_idx_ = (path_idx_ + 1) % profile_.attack_push.size();
    goal_sent_ = false;
    return;
  }
  if (ctx.double_vulnerability_active()) switch_stance(StanceCommand::ENHANCED_OFFENSIVE, now_s);
  drive_route(profile_.attack_push, now_s);
}

void DecisionFsm::behave_resupply(const Context & ctx, double now_s)
{
  if (!goal_sent_) {
    publish_single_goal(profile_.supply);
    return;
  }
  if (ctx.on_supply_pad()) return;

  if (now_s - operation_started_s_ > profile_.thresholds.resupply_timeout_s) {
    if (
      !profile_.backup_supply_points.empty() && path_idx_ < profile_.backup_supply_points.size()) {
      goal_sent_ = false;
      publish_single_goal(profile_.backup_supply_points[path_idx_++]);
      operation_started_s_ = now_s;
      return;
    }
    supply_backup_exhausted_ = true;
  }
}

void DecisionFsm::behave_retreat(const Context & ctx, double now_s)
{
  if (!goal_sent_) {
    publish_single_goal(profile_.retreat);
    return;
  }
  if (ctx.hp() < ctx.max_hp() * 0.05) switch_stance(StanceCommand::ENHANCED_DEFENSIVE, now_s);

  if (ctx.goal_reached()) return;

  const bool nav_timeout =
    ctx.nav_stuck() || (now_s - operation_started_s_ > profile_.thresholds.retreat_timeout_s);
  if (nav_timeout) {
    if (
      !profile_.backup_retreat_points.empty() &&
      path_idx_ < profile_.backup_retreat_points.size()) {
      goal_sent_ = false;
      publish_single_goal(profile_.backup_retreat_points[path_idx_++]);
      operation_started_s_ = now_s;
    } else if (path_idx_ == profile_.backup_retreat_points.size()) {
      goal_sent_ = false;
      publish_single_goal(profile_.safe_cover);
      operation_started_s_ = now_s;
      path_idx_++;
    }
  }
}

void DecisionFsm::drive_route(const Route & route, double now_s)
{
  if (route.empty()) return;
  if (path_idx_ >= route.size()) path_idx_ = 0;

  const Waypoint & wp = route[path_idx_];
  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_started_s_ = now_s;
    waypoint_arrived_s_ = 0.0;
    return;
  }

  if (!goal_arrived_) {
    return;
  }

  if (now_s - waypoint_arrived_s_ >= wp.dwell_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    goal_arrived_ = false;
  }
}

void DecisionFsm::publish_single_goal(const Waypoint & wp)
{
  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_arrived_s_ = 0.0;
  }
}

void DecisionFsm::switch_stance(StanceCommand stance, double now_s, bool force)
{
  if (!send_stance_ || stance == StanceCommand::NONE) return;
  if (stance == last_commanded_stance_) return;

  constexpr double kStanceSwitchCooldownS = 5.0;
  if (
    !force && !can_bypass_stance_cooldown(stance) &&
    now_s - last_stance_command_s_ < kStanceSwitchCooldownS) {
    return;
  }

  send_stance_(stance);
  last_commanded_stance_ = stance;
  last_stance_command_s_ = now_s;
}

bool DecisionFsm::can_bypass_stance_cooldown(StanceCommand stance) const
{
  return stance == StanceCommand::DEFENSIVE || stance == StanceCommand::ENHANCED_DEFENSIVE;
}

}  // namespace sentry_decision
