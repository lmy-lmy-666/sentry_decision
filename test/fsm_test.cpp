// Copyright 2026 Boombroke
//
// Unit tests for simplified DecisionFsm.
//
#include "sentry_decision_sample/fsm.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "sentry_decision_sample/context.hpp"
#include "sentry_decision_sample/profile.hpp"
#include "sentry_decision_sample/types.hpp"

using sentry_decision_sample::Context;
using sentry_decision_sample::DecisionFsm;
using sentry_decision_sample::NavStatus;
using sentry_decision_sample::Profile;
using sentry_decision_sample::State;
using sentry_decision_sample::Waypoint;

namespace
{

Profile make_profile()
{
  Profile p;
  p.name = "test";
  p.thresholds.hp_low = 150;
  p.thresholds.hp_critical = 60;
  p.thresholds.hp_critical_exit = 120;
  p.thresholds.ammo_min = 1;
  p.thresholds.game_total_time = 420;
  p.thresholds.min_ticks_in_state = 1;   // fast tests
  p.thresholds.resupply_timeout_s = 30.0;
  p.thresholds.resupply_total_timeout_s = 120.0;
  p.thresholds.retreat_timeout_s = 30.0;
  p.thresholds.retreat_total_timeout_s = 60.0;
  p.patrol = {{1.0, 1.0, 5.0}, {2.0, 2.0, 5.0}};
  p.supply = {-1.0, -5.0, 0.0};
  return p;
}

rm_interfaces::msg::GameStatus game(uint8_t progress, int32_t remain)
{
  rm_interfaces::msg::GameStatus m;
  m.game_progress = progress;
  m.stage_remain_time = remain;
  return m;
}

rm_interfaces::msg::RobotStatus robot(uint16_t hp, uint16_t ammo)
{
  rm_interfaces::msg::RobotStatus m;
  m.current_hp = hp;
  m.maximum_hp = 400;
  m.projectile_allowance_17mm = ammo;
  return m;
}

rm_interfaces::msg::GameRobotHP outpost_hp(uint16_t hp)
{
  rm_interfaces::msg::GameRobotHP m;
  m.ally_outpost_hp = hp;
  return m;
}

struct Harness
{
  std::vector<Waypoint> goals;
  bool nav_cancelled{false};
  Context ctx;
  DecisionFsm fsm;

  explicit Harness(Profile p)
  : fsm(
      std::move(p),
      [this](const Waypoint & w) { goals.push_back(w); },
      [this]() { nav_cancelled = true; })
  {
    ctx.set_thresholds(make_profile().thresholds);
    ctx.set_now(0.0);
  }
};

constexpr uint8_t RUNNING = rm_interfaces::msg::GameStatus::RUNNING;

}  // namespace

// =============================================================================
//  Basic state transitions
// =============================================================================

TEST(DecisionFsm, IdleWhenGameNotRunning)
{
  Harness h{make_profile()};
  h.ctx.update(game(rm_interfaces::msg::GameStatus::PREPARATION, 400));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

TEST(DecisionFsm, PatrolByDefault)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, ResupplyWhenAmmoEmpty)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 0));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, ResupplyWhenHpLow)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, RetreatWhenHpCritical)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(40, 50));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RETREAT);
}

TEST(DecisionFsm, CriticalHpBeatsResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(30, 0));  // critical + empty ammo → RETREAT wins
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RETREAT);
}

TEST(DecisionFsm, GameEndForcesIdle)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::GAME_OVER, 0));
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

TEST(DecisionFsm, RefereeStaleForcesIdle)
{
  Harness h{make_profile()};
  h.ctx.set_now(0.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  h.ctx.set_now(4.0);
  h.fsm.tick(h.ctx, 4.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

// =============================================================================
//  Patrol routing
// =============================================================================

TEST(DecisionFsm, PatrolSendsFirstWaypoint)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);
  EXPECT_DOUBLE_EQ(h.goals[0].y, 1.0);
}

TEST(DecisionFsm, PatrolAdvancesAfterArrivalAndDwell)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.fsm.tick(h.ctx, 1.0);
  h.fsm.tick(h.ctx, 6.0);
  h.ctx.set_nav_status(NavStatus::MOVING);
  h.fsm.tick(h.ctx, 6.0);
  ASSERT_GE(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, 2.0);
}

TEST(DecisionFsm, PatrolDoesNotAdvanceBeforeArrival)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  h.fsm.tick(h.ctx, 100.0);  // stuck timeout triggers skip
  EXPECT_EQ(h.goals.size(), 1u);
}

TEST(DecisionFsm, NavStuckAdvancesPatrol)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  const auto first = h.goals.back();
  h.ctx.set_nav_status(NavStatus::FAILED);
  h.fsm.tick(h.ctx, 1.0);
  h.ctx.set_nav_status(NavStatus::MOVING);
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_GE(h.goals.size(), 2u);
  EXPECT_NE(h.goals.back().x, first.x);
}

// =============================================================================
//  Patrol tactical variants
// =============================================================================

TEST(DecisionFsm, PatrolUsesAggressiveRouteWhenOutpostAlive)
{
  Profile p = make_profile();
  p.patrol_aggressive = {{8.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost_hp(500));  // outpost alive
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 8.0);
}

TEST(DecisionFsm, PatrolUsesLateRouteInLastMinute)
{
  Profile p = make_profile();
  p.patrol_late = {{2.0, 1.0, 5.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 30));  // < 60 s
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 2.0);
}

// =============================================================================
//  Resupply
// =============================================================================

TEST(DecisionFsm, ResupplyExitAtFullHp)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  h.ctx.update(robot(400, 50));  // full hp + ammo
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_NE(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, ResupplyStaysWithAmmoEmpty)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 0));  // ammo=0, hp full
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  // hp recovers but ammo still 0
  h.ctx.update(robot(400, 0));
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, BackupSupplyPublishedAfterTimeout)
{
  Profile p = make_profile();
  p.backup_supply_points = {{-3.0, -5.0, 0.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  h.fsm.tick(h.ctx, 31.0);  // > resupply_timeout_s (30)
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, -3.0);
}

TEST(DecisionFsm, ResupplyDoesNotOscillate)
{
  // After supply exhaustion, RESUPPLY should not be re-entered during cooldown
  Profile p = make_profile();
  p.thresholds.supply_retry_cooldown_s = 15.0;
  p.thresholds.resupply_timeout_s = 1.0;  // fast exhaustion
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  h.fsm.tick(h.ctx, 2.0);  // timeout → exhausted
  //  supply_backup_exhausted_ now true, hp 140 < 120? no, so can leave
  // Actually hp(140) >= hp_critical_exit(120), so RESUPPLY releases
  // Next tick: needs_resupply (140 < 150), but in_cooldown → skip → PATROL
  h.fsm.tick(h.ctx, 3.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, ResupplyStaysAfterCooldownRetry)
{
  // After cooldown expires, re-entering RESUPPLY must get a full fresh
  // attempt — not bail to PATROL because of a stale exhausted flag.
  Profile p = make_profile();
  p.thresholds.supply_retry_cooldown_s = 15.0;
  p.thresholds.resupply_timeout_s = 1.0;   // fast exhaustion
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));

  // exhaust supply over several ticks
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  h.fsm.tick(h.ctx, 2.0);   // behave_resupply sets exhausted=true this tick
                             // but select_state ran first → still RESUPPLY
  h.fsm.tick(h.ctx, 3.0);   // select_state now sees exhausted=true → PATROL
  ASSERT_EQ(h.fsm.state(), State::PATROL);

  // cooldown expired, hp still 140 (in the [120,150) gap)
  h.fsm.tick(h.ctx, 20.0);  // 20-2=18 > 15s cooldown → re-enter RESUPPLY
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);

  // THE KEY CHECK: next tick must stay RESUPPLY, not bail to PATROL
  h.fsm.tick(h.ctx, 21.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY)
      << "supply_backup_exhausted_ must be cleared on re-entry; "
         "otherwise the FSM sees a fresh enter but a stale exhausted flag "
         "and kicks itself back to PATROL after a single tick";
}

// =============================================================================
//  Retreat
// =============================================================================

TEST(DecisionFsm, RetreatGoesToSupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(40, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RETREAT);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);
  EXPECT_DOUBLE_EQ(h.goals[0].y, -5.0);
}

TEST(DecisionFsm, RetreatRetriesSupplyOnTimeout)
{
  Profile p = make_profile();
  p.supply = {-1.0, -5.0, 0.0};
  p.thresholds.retreat_timeout_s = 1.0;
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(40, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);  // supply
  h.fsm.tick(h.ctx, 2.0);                // timeout → retry
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, -1.0);  // supply again
}

TEST(DecisionFsm, RetreatHysteresis)
{
  // Enter RETREAT at hp=59, leave at hp >= 120
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(59, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RETREAT);
  h.ctx.update(robot(100, 50));  // recovered but still below exit
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RETREAT);
  h.ctx.update(robot(120, 50));  // now above exit
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_NE(h.fsm.state(), State::RETREAT);
}

// =============================================================================
//  Boundary cases
// =============================================================================

TEST(DecisionFsm, HpBoundaryNotEnterRetreat)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(60, 50));  // == hp_critical (not <)
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_NE(h.fsm.state(), State::RETREAT);
}

TEST(DecisionFsm, HpBoundaryEnterRetreat)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(59, 50));  // < hp_critical
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RETREAT);
}

TEST(DecisionFsm, IdleCancelsNavigation)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_FALSE(h.nav_cancelled);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::PREPARATION, 400));
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_TRUE(h.nav_cancelled);
}

TEST(DecisionFsm, MidPatrolHpDropGoesResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  h.ctx.update(robot(140, 50));  // hp drops below hp_low
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
