// Copyright 2026 Boombroke
//
// Simplified FSM — 3 states, navigation-only.
//
// Priority chain (evaluated every tick, first match wins):
//   ① IDLE           — referee offline or game not running
//   ② RESUPPLY       — hp < hp_low (150) OR ammo ≤ ammo_low (50).
//                      Drive to supply pad, recover passively (heal + free
//                      +100/min ammo). Exit only when hp == max AND ammo ≥
//                      ammo_ok (100). Never gives up (covers respawn).
//   ③ OPENING_STRIKE — match start, once: drive to a firing spot and dwell so
//                      auto-aim can destroy the enemy outpost. Preempted by
//                      RESUPPLY (low hp/ammo). Consumed after dwell or preempt.
//   ④ PATROL         — default: follow patrol route
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
  (void)now_s;

  // ① IDLE — referee down or game stopped
  if (!ctx.referee_fresh() || !ctx.game_running()) {
    state_reason_ = "referee stale / game not running";
    return State::IDLE;
  }

  // ② RESUPPLY — stay until fully recovered (hysteresis).
  //    Exit only when hp == max AND ammo ≥ ammo_ok. This also covers respawn:
  //    a dead sentry reads hp 0 → stays in RESUPPLY → keeps heading home after
  //    it revives, until fully healed and rearmed.
  if (state_ == State::RESUPPLY) {
    if (!ctx.resupply_done()) {
      state_reason_ = "recovering (hp/ammo not yet full)";
      return State::RESUPPLY;
    }
  } else if (ctx.needs_resupply()) {
    // RESUPPLY — enter when hp low or ammo low. This also preempts an ongoing
    // OPENING_STRIKE; the strike is a once-per-match action, so being preempted
    // consumes it (opening_done_ is latched in on_exit / behave).
    if (state_ == State::OPENING_STRIKE) opening_done_ = true;
    state_reason_ = ctx.hp_low() ? "hp low" : "ammo low";
    return State::RESUPPLY;
  }

  // ③ OPENING_STRIKE — match start, once. Requires a configured firing spot.
  if (state_ == State::OPENING_STRIKE) {
    // stay until behave_opening_strike marks it done (dwell elapsed)
    if (!opening_done_) {
      state_reason_ = "opening strike (killing enemy outpost)";
      return State::OPENING_STRIKE;
    }
  } else if (!opening_done_ && profile_.has_opening_strike) {
    state_reason_ = "opening strike start";
    return State::OPENING_STRIKE;
  }

  // ④ Default — PATROL
  state_reason_ = "default";
  return State::PATROL;
}

// ============================================================================
//  can_leave_current_state — oscillation guard
// ============================================================================

bool DecisionFsm::can_leave_current_state(State next) const
{
  if (next == state_) return true;

  // IDLE (game not running) always preempts, and is always leavable.
  if (state_ == State::IDLE) return true;
  if (next == State::IDLE) return true;

  // RESUPPLY (low hp / low ammo) preempts PATROL immediately.
  if (next == State::RESUPPLY) return true;

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
  ticks_in_state_ = 0;

  switch (s) {
    case State::IDLE:
      if (cancel_nav_) cancel_nav_();
      break;
    case State::OPENING_STRIKE:
      opening_entered_s_ = now_s;
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
    case State::IDLE:            behave_idle(ctx, now_s);           break;
    case State::OPENING_STRIKE:  behave_opening_strike(ctx, now_s); break;
    case State::PATROL:          behave_patrol(ctx, now_s);         break;
    case State::RESUPPLY:        behave_resupply(ctx, now_s);       break;
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
//  behave_opening_strike — drive to the firing spot, dwell to let auto-aim
//  destroy the enemy outpost, then consume this once-per-match action.
// ============================================================================

void DecisionFsm::behave_opening_strike(const Context & ctx, double now_s)
{
  // Dwell elapsed → strike consumed; select_state will fall through to PATROL.
  if (now_s - opening_entered_s_ >= profile_.thresholds.opening_strike_duration_s) {
    opening_done_ = true;
    return;
  }

  // Drive to the firing spot once; hold there while auto-aim fires.
  if (!goal_sent_) {
    publish_single_goal(profile_.opening_strike, now_s);
    return;
  }

  // Nav stuck en route → retry (re-plan). We keep trying: the strike spot is in
  // our own half and should be reachable; if truly blocked, the dwell timer
  // still expires and we move on.
  if (ctx.nav_failed()) {
    goal_sent_ = false;
    publish_single_goal(profile_.opening_strike, now_s);
  }
}

// ============================================================================
//  behave_patrol — follow patrol route, switch variants tactically
// ============================================================================

void DecisionFsm::behave_patrol(const Context & ctx, double now_s)
{
  // Tactical route selection driven by our outpost:
  //   outpost alive     → patrol_aggressive (前压), if defined
  //   outpost destroyed → patrol (我方半场防守)
  const Route * route = &profile_.patrol;
  if (ctx.outpost_alive() && !profile_.patrol_aggressive.empty()) {
    route = &profile_.patrol_aggressive;
  }

  // Route switched since last tick → reset waypoint tracking so we don't
  // judge arrival/timeout of the OLD goal against the NEW route's indices,
  // and don't chase a stale goal. Republish from the new route's start.
  if (route != active_route_) {
    active_route_ = route;
    path_idx_ = 0;
    goal_sent_ = false;
    goal_arrived_ = false;
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
  // --- arrival at the supply pad: POSITION-based primary, RFID auxiliary ---
  // Primary: Nav2/odom arrival (goal_reached) — self-computed, does not depend
  //   on the serial driver's RFID field (which may be mis-mapped upstream).
  // Auxiliary: RFID confirmation (5-tick sliding window, ≥3 hits) — used only
  //   as a faster/extra confirmation if the referee RFID bit is wired correctly.
  // Either one confirms → stay put and recover passively (no goal re-issue,
  //   so no 30s target jitter once we're home).
  rfid_window_ = static_cast<uint8_t>((rfid_window_ << 1) & 0x1F);
  if (ctx.on_supply_pad()) rfid_window_ |= 1;
  const bool rfid_confirmed = __builtin_popcount(rfid_window_) >= 3;

  if (goal_arrived_ || rfid_confirmed) {
    return;  // arrived — stay put, healing + refilling ammo passively
  }

  // --- navigation: keep heading to the supply pad, never give up ---
  // The pad is the only safe destination. We rotate through backup points on
  // stuck/timeout, then wrap back to the primary — so a respawned sentry (hp
  // recovered from 0) always resumes navigating home instead of stalling.

  // Nav stuck → advance to next candidate immediately.
  if (ctx.nav_failed()) {
    advance_supply_target(now_s);
    return;
  }

  // First goal → primary supply point.
  if (!goal_sent_) {
    publish_single_goal(current_supply_target(), now_s);
    return;
  }

  // Single-point timeout → rotate to next candidate.
  if (now_s - operation_started_s_ > profile_.thresholds.resupply_timeout_s) {
    advance_supply_target(now_s);
  }
}

// ============================================================================
//  supply target rotation — primary pad + backups, cycling forever
// ============================================================================

const Waypoint & DecisionFsm::current_supply_target() const
{
  // index 0 = primary supply pad; 1.. = backup_supply_points
  if (supply_backup_idx_ == 0 || profile_.backup_supply_points.empty()) {
    return profile_.supply;
  }
  const std::size_t i = (supply_backup_idx_ - 1) % profile_.backup_supply_points.size();
  return profile_.backup_supply_points[i];
}

void DecisionFsm::advance_supply_target(double now_s)
{
  // Cycle: primary → backup[0] → … → backup[n-1] → primary → …
  const std::size_t total = profile_.backup_supply_points.size() + 1;  // +1 for primary
  supply_backup_idx_ = (supply_backup_idx_ + 1) % total;
  goal_sent_ = false;
  publish_single_goal(current_supply_target(), now_s);
  operation_started_s_ = now_s;
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
