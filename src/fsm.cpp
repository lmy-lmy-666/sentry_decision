// Copyright 2026 Boombroke
//
// Simplified FSM — 4 states, navigation-only.
//
// Priority chain (evaluated every tick, first match wins):
//   ① IDLE      — referee offline or game not running
//   ② RETREAT   — hp < critical (60). Exit when hp ≥ critical_exit (120)
//   ③ RESUPPLY  — ammo empty or hp < low (150). Exit when full.
//   ④ PATROL    — default: follow patrol route
//
#include "sentry_decision_sample/fsm.hpp"

#include <utility>

namespace sentry_decision_sample
{

DecisionFsm::DecisionFsm(
  Profile profile, GoalPublisher goal_pub, NavCanceller nav_cancel)
: profile_(std::move(profile)),
  publish_goal_(std::move(goal_pub)),
  cancel_nav_(std::move(nav_cancel))
{
}

// ============================================================================
//  tick
// ============================================================================

void DecisionFsm::tick(const Context & ctx, double now_s)
{
  const State candidate = select_state(ctx, now_s);
  const State next = can_leave_current_state(candidate) ? candidate : state_;

  if (next != state_) {
    on_exit(state_);
    state_ = next;
    on_enter(next, now_s);
  }

  ++ticks_in_state_;
  run_behaviour(ctx, now_s);
}

// ============================================================================
//  select_state — priority chain
// ============================================================================

State DecisionFsm::select_state(const Context & ctx, double now_s)
{
  // ① IDLE — referee down or game stopped
  if (!ctx.referee_fresh() || !ctx.game_running()) {
    state_reason_ = "referee stale / game not running";
    return State::IDLE;
  }

  // ② RETREAT — critical hp (with hysteresis)
  if (state_ == State::RETREAT) {
    if (ctx.hp() < profile_.thresholds.hp_critical_exit) {
      state_reason_ = "hp still below retreat exit";
      return State::RETREAT;
    }
  } else if (ctx.hp_critical()) {
    state_reason_ = "hp critical";
    return State::RETREAT;
  }

  // ③ RESUPPLY — stay if still needed
  if (state_ == State::RESUPPLY) {
    if (ctx.ammo_empty()) {
      state_reason_ = "ammo empty, must stay in RESUPPLY";
      return State::RESUPPLY;
    }

    if (supply_backup_exhausted_) {
      if (ctx.hp() < profile_.thresholds.hp_critical_exit) {
        state_reason_ = "supply exhausted, hp still unsafe";
        return State::RESUPPLY;
      }
    } else {
      if (ctx.hp() < ctx.max_hp()) {
        state_reason_ = "still healing";
        return State::RESUPPLY;
      }
    }
  }

  // ④ RESUPPLY — enter if needed (cooldown check via helper)
  if (state_ != State::RESUPPLY && ctx.needs_resupply()) {
    // Ammo-empty bypasses cooldown — no ammo means can't do anything else
    if (!ctx.ammo_empty() && supply_in_cooldown(now_s)) {
      state_reason_ = "needs resupply but in cooldown";
    } else {
      state_reason_ = ctx.ammo_empty() ? "ammo empty" : "hp low";
      return State::RESUPPLY;
    }
  }

  // ⑤ Default — PATROL
  state_reason_ = "default";
  return State::PATROL;
}

bool DecisionFsm::supply_in_cooldown(double now_s) const
{
  return supply_backup_exhausted_ &&
         (supply_fail_time_ > 0.0) &&
         (now_s - supply_fail_time_ < profile_.thresholds.supply_retry_cooldown_s);
}

// ============================================================================
//  can_leave_current_state — oscillation guard
// ============================================================================

bool DecisionFsm::can_leave_current_state(State next) const
{
  if (next == state_) return true;

  // Safety states always preempt
  if (state_ == State::IDLE) return true;
  if (next == State::IDLE || next == State::RETREAT) return true;

  // RESUPPLY can preempt any non-RETREAT state
  if (next == State::RESUPPLY && state_ != State::RETREAT) return true;

  // Otherwise require minimum ticks to dampen oscillation
  return ticks_in_state_ >= profile_.thresholds.min_ticks_in_state;
}

// ============================================================================
//  on_enter / on_exit
// ============================================================================

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
    case State::RETREAT:
      operation_started_s_ = now_s;
      // Clear any stale supply cooldown — retreat destination IS the supply pad,
      // so when RETREAT ends we must be able to transition cleanly into RESUPPLY.
      supply_backup_exhausted_ = false;
      supply_fail_time_ = 0.0;
      break;
    case State::RESUPPLY:
      operation_started_s_ = now_s;
      rfid_window_ = 0;
      supply_backup_idx_ = 0;
      break;
    case State::PATROL:
      break;
  }
}

void DecisionFsm::on_exit(State)
{
}

// ============================================================================
//  run_behaviour — dispatch
// ============================================================================

void DecisionFsm::run_behaviour(const Context & ctx, double now_s)
{
  // Detect goal arrival (Nav2 feedback or fallback odom check)
  if (goal_sent_ && !goal_arrived_ && ctx.goal_reached()) {
    goal_arrived_ = true;
    waypoint_arrived_s_ = now_s;
  }

  switch (state_) {
    case State::IDLE:     behave_idle(ctx, now_s);     break;
    case State::PATROL:   behave_patrol(ctx, now_s);   break;
    case State::RESUPPLY: behave_resupply(ctx, now_s); break;
    case State::RETREAT:  behave_retreat(ctx, now_s);  break;
  }
}

// ============================================================================
//  behave_idle
// ============================================================================

void DecisionFsm::behave_idle(const Context &, double)
{
  // Navigation was cancelled in on_enter.  Nothing to do while idle.
}

// ============================================================================
//  behave_patrol — follow patrol route, switch variants tactically
// ============================================================================

void DecisionFsm::behave_patrol(const Context & ctx, double now_s)
{
  // Tactical route selection:
  //   late_game → patrol_late (if defined)
  //   outpost alive → patrol_aggressive (if defined)
  //   otherwise → default patrol
  const Route * route = &profile_.patrol;

  if (ctx.late_game() && !profile_.patrol_late.empty()) {
    route = &profile_.patrol_late;
  } else if (ctx.outpost_alive() && !profile_.patrol_aggressive.empty()) {
    route = &profile_.patrol_aggressive;
  }

  if (route->empty()) {
    goal_sent_ = true;  // prevent busy-looping
    return;
  }

  if (ctx.nav_failed()) {
    path_idx_ = (path_idx_ + 1) % route->size();
    goal_sent_ = false;
    return;
  }

  drive_route(*route, now_s);
}

// ============================================================================
//  behave_resupply — navigate to supply pad, heal, then leave
// ============================================================================

void DecisionFsm::behave_resupply(const Context & ctx, double now_s)
{
  // RFID debounce: 5-tick sliding window, ≥3 hits → confirmed.
  // Tolerates occasional signal dropout without resetting.
  rfid_window_ = static_cast<uint8_t>((rfid_window_ << 1) & 0x1F);
  if (ctx.on_supply_pad()) rfid_window_ |= 1;
  if (__builtin_popcount(rfid_window_) >= 3) {
    supply_backup_exhausted_ = false;
    supply_fail_time_ = 0.0;
    return;  // stay put — healing passively on the pad
  }

  // --- total timeout: cap the entire RESUPPLY chain ---
  if (now_s - state_entered_s_ > profile_.thresholds.resupply_total_timeout_s) {
    supply_backup_exhausted_ = true;
    supply_fail_time_ = now_s;
    return;
  }

  // --- navigation ---

  // Nav stuck → immediate fallback to next backup point
  if (ctx.nav_failed() && !profile_.backup_supply_points.empty() &&
      supply_backup_idx_ < profile_.backup_supply_points.size()) {
    goal_sent_ = false;
    publish_single_goal(profile_.backup_supply_points[supply_backup_idx_++], now_s);
    operation_started_s_ = now_s;
    return;
  }

  // First goal → primary supply point
  if (!goal_sent_) {
    publish_single_goal(profile_.supply, now_s);
    return;
  }

  // Single-point timeout → try next backup
  if (now_s - operation_started_s_ > profile_.thresholds.resupply_timeout_s) {
    if (!profile_.backup_supply_points.empty() &&
        supply_backup_idx_ < profile_.backup_supply_points.size()) {
      goal_sent_ = false;
      publish_single_goal(profile_.backup_supply_points[supply_backup_idx_++], now_s);
      operation_started_s_ = now_s;
      return;
    }
    supply_backup_exhausted_ = true;
    supply_fail_time_ = now_s;
  }
}

// ============================================================================
//  behave_retreat — keep heading to supply pad, Nav2 handles path planning.
//  Supply pad is in home base (enemy-forbidden zone), always reachable.
// ============================================================================

void DecisionFsm::behave_retreat(const Context & ctx, double now_s)
{
  // Total timeout safety net — stop trying after cap
  if (now_s - state_entered_s_ > profile_.thresholds.retreat_total_timeout_s) {
    return;
  }

  if (!goal_sent_) {
    publish_single_goal(profile_.supply, now_s);
    return;
  }

  if (ctx.goal_reached()) return;

  // Stuck or single-point timeout → retry (Nav2 replans)
  if (ctx.nav_failed() ||
      now_s - operation_started_s_ > profile_.thresholds.retreat_timeout_s) {
    goal_sent_ = false;
  }
}

// ============================================================================
//  drive_route — sequential waypoint traversal with dwell
// ============================================================================

void DecisionFsm::drive_route(const Route & route, double now_s)
{
  if (route.empty()) return;
  if (path_idx_ >= route.size()) path_idx_ = 0;

  const Waypoint & wp = route[path_idx_];

  // Send goal
  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_started_s_ = now_s;
    waypoint_arrived_s_ = 0.0;
    return;
  }

  // Per-waypoint stuck timeout → skip to next
  if (!goal_arrived_ && now_s - waypoint_started_s_ > profile_.thresholds.stuck_timeout_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    return;
  }

  // Waiting for arrival
  if (!goal_arrived_) return;

  // Dwell at waypoint
  if (now_s - waypoint_arrived_s_ >= wp.dwell_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    goal_arrived_ = false;
  }
}

// ============================================================================
//  publish_single_goal — one-shot waypoint
// ============================================================================

void DecisionFsm::publish_single_goal(const Waypoint & wp, double now_s)
{
  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_started_s_ = now_s;
    waypoint_arrived_s_ = 0.0;
  }
}

}  // namespace sentry_decision_sample
