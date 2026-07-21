// Copyright 2026 Boombroke
//
// BtDecision — sentry decision as a behaviour tree (behaviour-equal to the FSM).
//
// The tree is ticked once per cycle. Its shape:
//
//   Root (ReactiveSequence, runs all four steps every tick)
//     ├── SelectionFallback (ReactiveFallback = priority arbitration)
//     │     ├── [referee down / game stopped]      → candidate = IDLE
//     │     ├── [bump crossing/settling in progress]→ candidate = BUMP_TRAVERSE
//     │     └── [business + bump-entry check]       → candidate = RESUPPLY/OPENING/PATROL/BUMP
//     ├── Commit  (oscillation guard + on_enter, sets state_)
//     ├── ArrivalDetect (latch goal arrival, = run_behaviour prologue)
//     └── DispatchFallback (run the committed branch's behaviour)
//           ├── [state==IDLE]           → behave_idle
//           ├── [state==OPENING_STRIKE] → behave_opening_strike
//           ├── [state==PATROL]         → behave_patrol
//           ├── [state==RESUPPLY]       → behave_resupply
//           └── [state==BUMP_TRAVERSE]  → BumpPhaseTree (5-phase ReactiveFallback)
//
// Every numeric threshold, priority order and edge case matches DecisionFsm.
//
#include "omni_behavior_sample/bt_decision.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace omni_behavior_sample
{

using bt::NodeStatus;

BtDecision::BtDecision(
  Profile profile, GoalPublisher goal_pub, NavCanceller nav_cancel,
  BumpVelPublisher bump_vel_pub)
: profile_(std::move(profile)),
  publish_goal_(std::move(goal_pub)),
  cancel_nav_(std::move(nav_cancel)),
  publish_bump_vel_(std::move(bump_vel_pub))
{
  bump_disabled_.assign(profile_.bump_segments.size(), false);
  build_tree();
}

// ============================================================================
//  tick — set per-tick inputs, then run the tree
// ============================================================================

void BtDecision::tick(const Context & ctx, double now_s)
{
  ctx_ = &ctx;
  now_s_ = now_s;
  root_->tick();
}

// ============================================================================
//  build_tree — wire the leaves (lambdas over `this`) into the shape above
// ============================================================================

void BtDecision::build_tree()
{
  using namespace bt;

  // --- 1) SelectionFallback: priority arbitration → sets candidate_ ---------
  auto selection = MakeReactiveFallback(
    "SelectionFallback",
    // ① IDLE — referee down or game stopped (highest priority).
    MakeReactiveSequence(
      "sel_idle",
      Condition("referee_down_or_stopped",
        [this] { return !ctx_->referee_fresh() || !ctx_->game_running(); }),
      Action("set_idle", [this] {
        candidate_ = State::IDLE;
        candidate_reason_ = "referee stale / game not running";
        return NodeStatus::SUCCESS;
      })),
    // ② BUMP_TRAVERSE already crossing/settling — stay until DONE settled/FAILED.
    MakeReactiveSequence(
      "sel_bump_inprogress",
      Condition("bump_in_progress", [this] {
        if (state_ != State::BUMP_TRAVERSE) return false;
        const bool crossing =
          bump_phase_ != BumpPhase::DONE && bump_phase_ != BumpPhase::FAILED;
        const bool settling =
          bump_phase_ == BumpPhase::DONE &&
          bump_stop_frames_ < profile_.thresholds.bump_stop_ticks;
        return crossing || settling;
      }),
      Action("set_bump_inprogress", [this] {
        const bool crossing =
          bump_phase_ != BumpPhase::DONE && bump_phase_ != BumpPhase::FAILED;
        candidate_ = State::BUMP_TRAVERSE;
        candidate_reason_ =
          crossing ? "crossing undulating segment" : "settling after crossing";
        return NodeStatus::SUCCESS;
      })),
    // ③ business (RESUPPLY / OPENING_STRIKE / PATROL) + bump-entry check.
    Action("select_business_and_bump", [this] {
      select_business_and_bump();
      return NodeStatus::SUCCESS;
    }));

  // --- 2) Commit: oscillation guard + on_enter -----------------------------
  auto commit = Action("commit", [this] {
    const State next = can_leave_current_state(candidate_) ? candidate_ : state_;
    if (next != state_) {
      // on_exit is a no-op in the FSM sample; kept as a comment for parity.
      state_ = next;
      on_enter(next);
    }
    ++ticks_in_state_;
    state_reason_ = candidate_reason_;
    return NodeStatus::SUCCESS;
  });

  // --- 3) ArrivalDetect: latch goal arrival (= run_behaviour prologue) ------
  auto arrival = Action("arrival_detect", [this] {
    if (goal_sent_ && !goal_arrived_ && ctx_->goal_reached()) {
      goal_arrived_ = true;
      waypoint_arrived_s_ = now_s_;
    }
    return NodeStatus::SUCCESS;
  });

  // --- BUMP phase subtree: 5-phase machine as a ReactiveFallback -----------
  // Each branch guards on the current phase; the handler returns SUCCESS to
  // advance-and-re-evaluate THIS tick (FSM `continue`) or RUNNING to end the
  // tick (FSM `return`). behave_bump() below loop-ticks this subtree. We keep a
  // raw pointer to re-tick it directly; ownership stays in the dispatch branch.
  bump_phase_tree_ = MakeReactiveFallback(
    "BumpPhaseFallback",
    MakeReactiveSequence("ph_goto_entry",
      Condition("is_goto_entry", [this] { return bump_phase_ == BumpPhase::GOTO_ENTRY; }),
      Action("do_goto_entry", [this] { return bump_goto_entry(); })),
    MakeReactiveSequence("ph_align",
      Condition("is_align", [this] { return bump_phase_ == BumpPhase::ALIGN; }),
      Action("do_align", [this] { return bump_align(); })),
    MakeReactiveSequence("ph_dashing",
      Condition("is_dashing", [this] { return bump_phase_ == BumpPhase::DASHING; }),
      Action("do_dashing", [this] { return bump_dashing(); })),
    MakeReactiveSequence("ph_done",
      Condition("is_done", [this] { return bump_phase_ == BumpPhase::DONE; }),
      Action("do_done", [this] { return bump_done(); })),
    MakeReactiveSequence("ph_failed",
      Condition("is_failed", [this] { return bump_phase_ == BumpPhase::FAILED; }),
      Action("do_failed", [this] { return bump_failed(); })));

  // --- 4) DispatchFallback: run the committed branch's behaviour ------------
  auto dispatch = MakeReactiveFallback(
    "DispatchFallback",
    MakeReactiveSequence("disp_idle",
      Condition("is_idle", [this] { return state_ == State::IDLE; }),
      Action("behave_idle", [this] { behave_idle(); return NodeStatus::SUCCESS; })),
    MakeReactiveSequence("disp_opening",
      Condition("is_opening", [this] { return state_ == State::OPENING_STRIKE; }),
      Action("behave_opening", [this] { behave_opening_strike(); return NodeStatus::SUCCESS; })),
    MakeReactiveSequence("disp_patrol",
      Condition("is_patrol", [this] { return state_ == State::PATROL; }),
      Action("behave_patrol", [this] { behave_patrol(); return NodeStatus::SUCCESS; })),
    MakeReactiveSequence("disp_resupply",
      Condition("is_resupply", [this] { return state_ == State::RESUPPLY; }),
      Action("behave_resupply", [this] { behave_resupply(); return NodeStatus::SUCCESS; })),
    MakeReactiveSequence("disp_bump",
      Condition("is_bump", [this] { return state_ == State::BUMP_TRAVERSE; }),
      Action("behave_bump", [this] {
        // FSM's for(;;) loop: re-tick the phase subtree while a handler
        // advanced the phase (SUCCESS = continue); stop on RUNNING (= return).
        for (;;) {
          if (bump_phase_tree_->tick() == NodeStatus::RUNNING) break;
        }
        return NodeStatus::SUCCESS;
      })));

  // --- root: run all four steps every tick, in order -----------------------
  root_ = MakeReactiveSequence(
    "Root",
    std::move(selection), std::move(commit),
    std::move(arrival), std::move(dispatch));
}

// ============================================================================
//  select_business_and_bump — the ③ business logic + bump-entry check
// ============================================================================

void BtDecision::select_business_and_bump()
{
  // Determine the business state (RESUPPLY / OPENING_STRIKE / PATROL) and its
  // destination x, then check if a bump segment sits between us and it.

  State   business = State::PATROL;
  double  goal_x   = 0.0;
  const char * reason = "default";

  // ③ RESUPPLY — stay until fully recovered (hysteresis).
  if (state_ == State::RESUPPLY && !ctx_->resupply_done()) {
    business = State::RESUPPLY;
    goal_x   = current_supply_target().x;
    reason   = "recovering (hp/ammo not yet full)";
  } else if (state_ != State::RESUPPLY && ctx_->needs_resupply()) {
    // Enter RESUPPLY when hp/ammo low. Also preempts (consumes) OPENING_STRIKE.
    if (state_ == State::OPENING_STRIKE) opening_done_ = true;
    business = State::RESUPPLY;
    goal_x   = current_supply_target().x;
    reason   = ctx_->hp_low() ? "hp low" : "ammo low";
  } else if (state_ == State::OPENING_STRIKE && !opening_done_) {
    business = State::OPENING_STRIKE;
    goal_x   = profile_.opening_strike.x;
    reason   = "opening strike (killing enemy outpost)";
  } else if (state_ != State::OPENING_STRIKE && !opening_done_ && profile_.has_opening_strike) {
    business = State::OPENING_STRIKE;
    goal_x   = profile_.opening_strike.x;
    reason   = "opening strike start";
  } else {
    business = State::PATROL;
    const Route & r = (ctx_->outpost_alive() && !profile_.patrol_aggressive.empty())
                        ? profile_.patrol_aggressive : profile_.patrol;
    goal_x = r.empty() ? ctx_->sentry_x() : r.front().x;
    reason = "default";
  }

  // ④ BUMP_TRAVERSE — if an undulating segment separates us from goal_x, cross
  //    it first. OPENING_STRIKE is never routed through a bump (opening spot is
  //    in our own half); only PATROL and RESUPPLY may need to cross.
  if (business != State::OPENING_STRIKE) {
    const int seg = find_bump_to_cross(goal_x);
    if (seg >= 0) {
      bump_seg_idx_ = seg;
      const BumpSegment & bs = profile_.bump_segments[seg];
      const double lo = std::min(bs.entry.x, bs.exit.x);
      const bool robot_low = ctx_->sentry_x() <= lo;
      const bool exit_is_high = bs.exit.x >= bs.entry.x;
      bump_dir_ = (robot_low == exit_is_high) ? BumpDir::FORWARD : BumpDir::BACKWARD;
      candidate_ = State::BUMP_TRAVERSE;
      candidate_reason_ = "bump segment between us and goal";
      return;
    }
  }

  candidate_ = business;
  candidate_reason_ = reason;
}

// ============================================================================
//  find_bump_to_cross
// ============================================================================

int BtDecision::find_bump_to_cross(double goal_x) const
{
  if (!ctx_->sentry_pos_valid() || profile_.bump_segments.empty()) return -1;

  const double rx = ctx_->sentry_x();
  const double ry = ctx_->sentry_y();

  for (std::size_t i = 0; i < profile_.bump_segments.size(); ++i) {
    if (bump_disabled_[i]) continue;
    const BumpSegment & s = profile_.bump_segments[i];

    const double lo = std::min(s.entry.x, s.exit.x);
    const double hi = std::max(s.entry.x, s.exit.x);

    if (std::abs(ry - s.entry.y) > profile_.thresholds.bump_entry_radius) continue;

    const int robot_side = rx < lo ? -1 : (rx > hi ? +1 : 0);
    const int goal_side  = goal_x < lo ? -1 : (goal_x > hi ? +1 : 0);
    if (robot_side == 0 || goal_side == 0 || robot_side == goal_side) continue;

    return static_cast<int>(i);
  }
  return -1;
}

// ============================================================================
//  can_leave_current_state — oscillation guard
// ============================================================================

bool BtDecision::can_leave_current_state(State next) const
{
  if (next == state_) return true;

  if (next == State::IDLE) return true;
  if (state_ == State::IDLE) return true;

  if (state_ == State::BUMP_TRAVERSE) {
    return bump_phase_ == BumpPhase::DONE || bump_phase_ == BumpPhase::FAILED;
  }

  if (next == State::RESUPPLY) return true;

  return ticks_in_state_ >= profile_.thresholds.min_ticks_in_state;
}

// ============================================================================
//  on_enter
// ============================================================================

void BtDecision::on_enter(State s)
{
  path_idx_ = 0;
  goal_sent_ = false;
  goal_arrived_ = false;
  waypoint_started_s_ = now_s_;
  waypoint_arrived_s_ = 0.0;
  ticks_in_state_ = 0;

  switch (s) {
    case State::IDLE:
      if (cancel_nav_) cancel_nav_();
      break;
    case State::OPENING_STRIKE:
      opening_entered_s_ = now_s_;
      break;
    case State::RESUPPLY:
      operation_started_s_ = now_s_;
      rfid_window_ = 0;
      supply_backup_idx_ = 0;
      break;
    case State::PATROL:
      break;
    case State::BUMP_TRAVERSE:
      bump_phase_ = BumpPhase::GOTO_ENTRY;
      bump_phase_started_s_ = now_s_;
      bump_last_vx_ = 0.0;
      bump_stop_frames_ = 0;
      break;
  }
}

// ============================================================================
//  behave_idle
// ============================================================================

void BtDecision::behave_idle()
{
  // Navigation was cancelled in on_enter. Nothing to do while idle.
}

// ============================================================================
//  behave_opening_strike
// ============================================================================

void BtDecision::behave_opening_strike()
{
  if (now_s_ - opening_entered_s_ >= profile_.thresholds.opening_strike_duration_s) {
    opening_done_ = true;
    return;
  }

  if (!goal_sent_) {
    publish_single_goal(profile_.opening_strike);
    return;
  }

  if (ctx_->nav_failed()) {
    goal_sent_ = false;
    publish_single_goal(profile_.opening_strike);
  }
}

// ============================================================================
//  behave_patrol
// ============================================================================

void BtDecision::behave_patrol()
{
  const Route * route = &profile_.patrol;
  if (ctx_->outpost_alive() && !profile_.patrol_aggressive.empty()) {
    route = &profile_.patrol_aggressive;
  }

  if (route != active_route_) {
    active_route_ = route;
    path_idx_ = 0;
    goal_sent_ = false;
    goal_arrived_ = false;
  }

  if (route->empty()) {
    goal_sent_ = true;
    return;
  }

  if (ctx_->nav_failed()) {
    path_idx_ = (path_idx_ + 1) % route->size();
    goal_sent_ = false;
    return;
  }

  drive_route(*route);
}

// ============================================================================
//  behave_resupply
// ============================================================================

void BtDecision::behave_resupply()
{
  rfid_window_ = static_cast<uint8_t>((rfid_window_ << 1) & 0x1F);
  if (ctx_->on_supply_pad()) rfid_window_ |= 1;
  const bool rfid_confirmed = __builtin_popcount(rfid_window_) >= 3;

  if (goal_arrived_ || rfid_confirmed) {
    return;
  }

  if (ctx_->nav_failed()) {
    advance_supply_target();
    return;
  }

  if (!goal_sent_) {
    publish_single_goal(current_supply_target());
    return;
  }

  if (now_s_ - operation_started_s_ > profile_.thresholds.resupply_timeout_s) {
    advance_supply_target();
  }
}

// ============================================================================
//  bump phase handlers
// ============================================================================

NodeStatus BtDecision::bump_goto_entry()
{
  if (!goal_sent_) {
    publish_single_goal(bump_entry_target());
    return NodeStatus::RUNNING;
  }
  const double dx = ctx_->sentry_x() - bump_entry_target().x;
  const double dy = ctx_->sentry_y() - bump_entry_target().y;
  const bool near_entry = std::hypot(dx, dy) <= profile_.thresholds.bump_entry_radius;
  if (goal_arrived_ || near_entry) {
    if (cancel_nav_) cancel_nav_();
    bump_phase_ = BumpPhase::ALIGN;
    bump_phase_started_s_ = now_s_;
    return NodeStatus::RUNNING;
  }
  if (ctx_->nav_failed()) {
    goal_sent_ = false;
    publish_single_goal(bump_entry_target());
  }
  return NodeStatus::RUNNING;
}

NodeStatus BtDecision::bump_align()
{
  if (now_s_ - bump_phase_started_s_ >= profile_.thresholds.bump_align_time_s) {
    bump_phase_ = BumpPhase::DASHING;
    bump_phase_started_s_ = now_s_;
    bump_last_vx_ = bump_dash_vx();
    return NodeStatus::SUCCESS;  // continue to DASHING this tick
  }
  if (publish_bump_vel_) publish_bump_vel_(0.0);
  return NodeStatus::RUNNING;
}

NodeStatus BtDecision::bump_dashing()
{
  if (now_s_ - bump_phase_started_s_ > profile_.thresholds.bump_timeout_s) {
    bump_phase_ = BumpPhase::FAILED;
    bump_phase_started_s_ = now_s_;
    return NodeStatus::SUCCESS;
  }
  if (!ctx_->sentry_pos_valid()) {
    if (publish_bump_vel_) publish_bump_vel_(bump_last_vx_);
    return NodeStatus::RUNNING;
  }
  if (bump_reached()) {
    if (publish_bump_vel_) publish_bump_vel_(0.0);
    bump_phase_ = BumpPhase::DONE;
    bump_stop_frames_ = 0;
    return NodeStatus::RUNNING;
  }
  bump_last_vx_ = bump_dash_vx();
  if (publish_bump_vel_) publish_bump_vel_(bump_last_vx_);
  return NodeStatus::RUNNING;
}

NodeStatus BtDecision::bump_done()
{
  if (publish_bump_vel_) publish_bump_vel_(0.0);
  ++bump_stop_frames_;
  return NodeStatus::RUNNING;
}

NodeStatus BtDecision::bump_failed()
{
  const double target_x = bump_entry_target().x;
  const double tol = profile_.thresholds.bump_tol;
  const double rv = profile_.thresholds.bump_reverse_speed;
  const double vx = (target_x >= bump_dash_target().x) ? rv : -rv;

  if (now_s_ - bump_phase_started_s_ > profile_.thresholds.bump_timeout_s) {
    if (publish_bump_vel_) publish_bump_vel_(0.0);
    if (bump_seg_idx_ >= 0) bump_disabled_[bump_seg_idx_] = true;
    return NodeStatus::RUNNING;
  }
  if (!ctx_->sentry_pos_valid()) {
    if (publish_bump_vel_) publish_bump_vel_(vx);
    return NodeStatus::RUNNING;
  }
  const bool back = (vx > 0.0) ? (ctx_->sentry_x() >= target_x - tol)
                               : (ctx_->sentry_x() <= target_x + tol);
  if (back) {
    if (publish_bump_vel_) publish_bump_vel_(0.0);
    if (bump_seg_idx_ >= 0) bump_disabled_[bump_seg_idx_] = true;
    return NodeStatus::RUNNING;
  }
  if (publish_bump_vel_) publish_bump_vel_(vx);
  return NodeStatus::RUNNING;
}

// ============================================================================
//  bump helpers
// ============================================================================

const Waypoint & BtDecision::bump_entry_target() const
{
  const BumpSegment & s = profile_.bump_segments[bump_seg_idx_];
  return (bump_dir_ == BumpDir::FORWARD) ? s.entry : s.exit;
}

const Waypoint & BtDecision::bump_dash_target() const
{
  const BumpSegment & s = profile_.bump_segments[bump_seg_idx_];
  return (bump_dir_ == BumpDir::FORWARD) ? s.exit : s.entry;
}

double BtDecision::bump_dash_vx() const
{
  const double target_x = bump_dash_target().x;
  const double from_x   = bump_entry_target().x;
  const double v = profile_.thresholds.bump_dash_speed;
  return (target_x >= from_x) ? v : -v;
}

bool BtDecision::bump_reached() const
{
  const double target_x = bump_dash_target().x;
  const double tol = profile_.thresholds.bump_tol;
  return (bump_dash_vx() > 0.0) ? (ctx_->sentry_x() >= target_x - tol)
                                : (ctx_->sentry_x() <= target_x + tol);
}

// ============================================================================
//  supply target rotation
// ============================================================================

const Waypoint & BtDecision::current_supply_target() const
{
  if (supply_backup_idx_ == 0 || profile_.backup_supply_points.empty()) {
    return profile_.supply;
  }
  const std::size_t i = (supply_backup_idx_ - 1) % profile_.backup_supply_points.size();
  return profile_.backup_supply_points[i];
}

void BtDecision::advance_supply_target()
{
  const std::size_t total = profile_.backup_supply_points.size() + 1;
  supply_backup_idx_ = (supply_backup_idx_ + 1) % total;
  goal_sent_ = false;
  publish_single_goal(current_supply_target());
  operation_started_s_ = now_s_;
}

// ============================================================================
//  navigation helpers
// ============================================================================

void BtDecision::drive_route(const Route & route)
{
  if (route.empty()) return;
  if (path_idx_ >= route.size()) path_idx_ = 0;

  const Waypoint & wp = route[path_idx_];

  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_started_s_ = now_s_;
    waypoint_arrived_s_ = 0.0;
    return;
  }

  if (!goal_arrived_ && now_s_ - waypoint_started_s_ > profile_.thresholds.stuck_timeout_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    return;
  }

  if (!goal_arrived_) return;

  if (now_s_ - waypoint_arrived_s_ >= wp.dwell_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    goal_arrived_ = false;
  }
}

void BtDecision::publish_single_goal(const Waypoint & wp)
{
  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_started_s_ = now_s_;
    waypoint_arrived_s_ = 0.0;
  }
}

}  // namespace omni_behavior_sample
