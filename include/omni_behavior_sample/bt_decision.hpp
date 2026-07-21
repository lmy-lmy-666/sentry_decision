// Copyright 2026 Boombroke
//
// BtDecision — the sentry decision, expressed as a behaviour tree.
// ================================================================
// Same behaviour as the FSM sample (IDLE / OPENING_STRIKE / PATROL / RESUPPLY /
// BUMP_TRAVERSE, same thresholds, same priorities, same edge cases), but the
// control flow is a behaviour tree instead of a hand-rolled state switch.
//
// How the FSM maps onto the tree
// ------------------------------
// The FSM tick did three things in order: pick a candidate state (priority
// chain), commit to it (with an oscillation guard), then run that state's
// behaviour. The tree keeps the SAME three phases, each now a proper subtree:
//
//   Root = Sequence
//     1. SelectionFallback  (ReactiveFallback, priority order)   → sets candidate
//     2. Commit             (Action)                             → applies guard, on_enter
//     3. DispatchFallback   (ReactiveFallback over committed)    → runs behaviour
//
// * SelectionFallback is a priority arbitration exactly like the FSM chain:
//     IDLE  >  BUMP(in-progress)  >  {RESUPPLY / OPENING / PATROL + bump-entry}
//   The first applicable branch wins, higher priority preempts lower — which is
//   what a ReactiveFallback does natively.
// * Commit reproduces can_leave_current_state (min-ticks oscillation damping,
//   RESUPPLY/IDLE immediate preempt, BUMP preempt-proof-until-DONE) and the
//   per-state on_enter reset.
// * DispatchFallback routes to the committed state's behaviour. BUMP_TRAVERSE's
//   five phases (GOTO_ENTRY→ALIGN→DASHING→DONE/FAILED) are themselves a
//   ReactiveFallback subtree — the part of the design that benefits most from
//   being a tree.
//
// All progress fields (route cursor, goal-sent latch, bump phase, supply
// rotation, …) live here as members and are the tree's blackboard; leaves are
// lambdas closing over `this`. The public surface (ctor callbacks, tick,
// state(), state_reason(), profile()) is identical to DecisionFsm so the ROS
// node and the unit tests are drop-in.
//
#ifndef OMNI_BEHAVIOR_SAMPLE__BT_DECISION_HPP_
#define OMNI_BEHAVIOR_SAMPLE__BT_DECISION_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "omni_behavior_sample/behavior_tree.hpp"
#include "omni_behavior_sample/context.hpp"
#include "omni_behavior_sample/profile.hpp"
#include "omni_behavior_sample/types.hpp"

namespace omni_behavior_sample
{

using GoalPublisher = std::function<void(const Waypoint &)>;
using NavCanceller  = std::function<void()>;
/// Publish an open-loop chassis velocity (linear.x only) while crossing a bump
/// segment. vx is signed (m/s); y and yaw are always zero (swerve constraint).
using BumpVelPublisher = std::function<void(double vx)>;

class BtDecision
{
public:
  BtDecision(
    Profile profile, GoalPublisher goal_pub, NavCanceller nav_cancel = nullptr,
    BumpVelPublisher bump_vel_pub = nullptr);

  /// Main entry point — call at tick_frequency (default 10 Hz).
  void tick(const Context & ctx, double now_s);

  State state() const { return state_; }
  const char * state_reason() const { return state_reason_; }
  const Profile & profile() const { return profile_; }

private:
  // --- tree construction ------------------------------------------
  void build_tree();

  // --- selection (candidate arbitration = FSM select_state) -------
  void select_business_and_bump();     ///< the RESUPPLY/OPENING/PATROL + bump-entry logic

  // --- commit (= FSM can_leave_current_state + on_enter) ----------
  bool can_leave_current_state(State next) const;
  void on_enter(State s);

  // --- behaviours (= FSM behave_*) --------------------------------
  void behave_idle();
  void behave_opening_strike();
  void behave_patrol();
  void behave_resupply();

  // bump phase handlers: return SUCCESS to re-evaluate the phase subtree THIS
  // tick (the FSM's `continue`), RUNNING to end the tick (the FSM's `return`).
  bt::NodeStatus bump_goto_entry();
  bt::NodeStatus bump_align();
  bt::NodeStatus bump_dashing();
  bt::NodeStatus bump_done();
  bt::NodeStatus bump_failed();

  // --- bump traverse helpers (identical to FSM) -------------------
  int  find_bump_to_cross(double goal_x) const;
  const Waypoint & bump_dash_target() const;
  const Waypoint & bump_entry_target() const;
  double bump_dash_vx() const;
  bool   bump_reached() const;

  // --- navigation helpers -----------------------------------------
  void drive_route(const Route & route);
  void publish_single_goal(const Waypoint & wp);

  // --- supply target rotation -------------------------------------
  const Waypoint & current_supply_target() const;
  void advance_supply_target();

  // ==================================================================
  //  members  (= blackboard)
  // ==================================================================

  Profile          profile_;
  GoalPublisher    publish_goal_;
  NavCanceller     cancel_nav_;
  BumpVelPublisher publish_bump_vel_;

  // the behaviour tree
  bt::TreeNodePtr root_;
  // BUMP_TRAVERSE 5-phase subtree. Owned separately (not parked in root_) so the
  // dispatch bump branch can loop-tick it directly, matching the FSM's for(;;).
  bt::TreeNodePtr bump_phase_tree_;

  // per-tick inputs (set at the top of tick(); read by leaves)
  const Context * ctx_{nullptr};
  double now_s_{0.0};

  // committed branch label + the candidate chosen by SelectionFallback
  State state_{State::IDLE};
  State candidate_{State::IDLE};
  const char * candidate_reason_{""};

  // route tracking
  std::size_t   path_idx_{0};
  bool          goal_sent_{false};
  const Route * active_route_{nullptr};
  double      waypoint_started_s_{0.0};
  double      waypoint_arrived_s_{0.0};
  bool        goal_arrived_{false};

  // state timing
  int    ticks_in_state_{0};

  // OPENING_STRIKE
  bool   opening_done_{false};
  double opening_entered_s_{0.0};

  // RESUPPLY
  uint8_t     rfid_window_{0};
  std::size_t supply_backup_idx_{0};
  double      operation_started_s_{0.0};

  // BUMP_TRAVERSE
  BumpPhase bump_phase_{BumpPhase::GOTO_ENTRY};
  int       bump_seg_idx_{-1};
  BumpDir   bump_dir_{BumpDir::FORWARD};
  double    bump_phase_started_s_{0.0};
  double    bump_last_vx_{0.0};
  int       bump_stop_frames_{0};
  std::vector<bool> bump_disabled_;

  // logging
  const char * state_reason_{""};
};

}  // namespace omni_behavior_sample

#endif  // OMNI_BEHAVIOR_SAMPLE__BT_DECISION_HPP_
