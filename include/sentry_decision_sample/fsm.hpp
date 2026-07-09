// Copyright 2026 Boombroke
//
// Simplified FSM — 4 states, no combat, no stance.
// IDLE → RETREAT → RESUPPLY → PATROL (priority order).
//
#ifndef SENTRY_DECISION_SAMPLE__FSM_HPP_
#define SENTRY_DECISION_SAMPLE__FSM_HPP_

#include <cstddef>
#include <functional>

#include "sentry_decision_sample/context.hpp"
#include "sentry_decision_sample/profile.hpp"
#include "sentry_decision_sample/types.hpp"

namespace sentry_decision_sample
{

using GoalPublisher = std::function<void(const Waypoint &)>;
using NavCanceller  = std::function<void()>;

class DecisionFsm
{
public:
  DecisionFsm(Profile profile, GoalPublisher goal_pub, NavCanceller nav_cancel = nullptr);

  /// Main entry point — call at tick_frequency (default 10 Hz).
  void tick(const Context & ctx, double now_s);

  State state() const { return state_; }
  const char * state_reason() const { return state_reason_; }
  const Profile & profile() const { return profile_; }

private:
  // --- state machine -----------------------------------------------
  State select_state(const Context & ctx, double now_s);
  bool  can_leave_current_state(State next) const;
  bool  supply_in_cooldown(double now_s) const;

  void on_enter(State s, double now_s);
  void on_exit(State s);
  void run_behaviour(const Context & ctx, double now_s);

  // --- behaviours --------------------------------------------------
  void behave_idle(const Context & ctx, double now_s);
  void behave_patrol(const Context & ctx, double now_s);
  void behave_resupply(const Context & ctx, double now_s);
  void behave_retreat(const Context & ctx, double now_s);

  // --- navigation helpers ------------------------------------------
  void drive_route(const Route & route, double now_s);
  void publish_single_goal(const Waypoint & wp, double now_s);

  // ==================================================================
  //  members
  // ==================================================================

  Profile       profile_;
  GoalPublisher publish_goal_;
  NavCanceller  cancel_nav_;

  State state_{State::IDLE};

  // route tracking
  std::size_t path_idx_{0};
  bool        goal_sent_{false};
  double      waypoint_started_s_{0.0};
  double      waypoint_arrived_s_{0.0};
  bool        goal_arrived_{false};

  // state timing
  double state_entered_s_{0.0};
  int    ticks_in_state_{0};

  // RESUPPLY state
  bool        supply_backup_exhausted_{false};
  double      supply_fail_time_{0.0};
  uint8_t     rfid_window_{0};        ///< RFID debounce: 5-tick sliding window, ≥3 hits → confirmed
  std::size_t supply_backup_idx_{0};  ///< current backup supply point index (own variable, not path_idx_)

  // RETREAT state
  double operation_started_s_{0.0};

  // logging
  const char * state_reason_{""};     ///< why was the current state selected
};

}  // namespace sentry_decision_sample

#endif  // SENTRY_DECISION_SAMPLE__FSM_HPP_
