// Copyright 2026 Boombroke
//
// Unit tests for the behaviour-tree decision (BtDecision).
// These are the SAME behavioural assertions as the FSM sample's fsm_test.cpp —
// only the engine type differs — proving the BT port is behaviour-equal.
//
#include "omni_behavior_sample/bt_decision.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "omni_behavior_sample/context.hpp"
#include "omni_behavior_sample/profile.hpp"
#include "omni_behavior_sample/types.hpp"

using omni_behavior_sample::BtDecision;
using omni_behavior_sample::Context;
using omni_behavior_sample::NavStatus;
using omni_behavior_sample::Profile;
using omni_behavior_sample::State;
using omni_behavior_sample::Waypoint;

namespace
{

Profile make_profile()
{
  Profile p;
  p.name = "test";
  p.thresholds.hp_low = 150;
  p.thresholds.ammo_low = 50;
  p.thresholds.ammo_ok = 100;
  p.thresholds.game_total_time = 420;
  p.thresholds.min_ticks_in_state = 1;   // fast tests
  p.thresholds.resupply_timeout_s = 30.0;
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
  BtDecision bt;

  explicit Harness(Profile p)
  : bt(
      std::move(p),
      [this](const Waypoint & w) { goals.push_back(w); },
      [this]() { nav_cancelled = true; })
  {
    ctx.set_thresholds(make_profile().thresholds);
    ctx.set_now(0.0);
  }
};

constexpr uint8_t RUNNING = rm_interfaces::msg::GameStatus::RUNNING;

// "healthy" = full hp + plenty of ammo → PATROL
constexpr uint16_t FULL_HP = 400;
constexpr uint16_t FULL_AMMO = 300;

}  // namespace

// =============================================================================
//  Basic branch transitions
// =============================================================================

TEST(BtDecision, IdleWhenGameNotRunning)
{
  Harness h{make_profile()};
  h.ctx.update(game(rm_interfaces::msg::GameStatus::PREPARATION, 400));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::IDLE);
}

TEST(BtDecision, PatrolByDefault)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BtDecision, ResupplyWhenAmmoLow)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, 50));  // ammo == ammo_low → enter
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::RESUPPLY);
}

TEST(BtDecision, ResupplyWhenHpLow)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));  // hp < hp_low
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::RESUPPLY);
}

TEST(BtDecision, GameEndForcesIdle)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::PATROL);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::GAME_OVER, 0));
  h.bt.tick(h.ctx, 1.0);
  EXPECT_EQ(h.bt.state(), State::IDLE);
}

TEST(BtDecision, RefereeStaleForcesIdle)
{
  Harness h{make_profile()};
  h.ctx.set_now(0.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::PATROL);
  h.ctx.set_now(4.0);
  h.bt.tick(h.ctx, 4.0);
  EXPECT_EQ(h.bt.state(), State::IDLE);
}

// =============================================================================
//  Patrol routing
// =============================================================================

TEST(BtDecision, PatrolSendsFirstWaypoint)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::PATROL);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);
  EXPECT_DOUBLE_EQ(h.goals[0].y, 1.0);
}

TEST(BtDecision, PatrolAdvancesAfterArrivalAndDwell)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.bt.tick(h.ctx, 1.0);
  h.bt.tick(h.ctx, 6.0);
  h.ctx.set_nav_status(NavStatus::MOVING);
  h.bt.tick(h.ctx, 6.0);
  ASSERT_GE(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, 2.0);
}

TEST(BtDecision, PatrolDoesNotAdvanceBeforeArrival)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  h.bt.tick(h.ctx, 100.0);  // stuck timeout triggers skip
  EXPECT_EQ(h.goals.size(), 1u);
}

TEST(BtDecision, NavStuckAdvancesPatrol)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::PATROL);
  const auto first = h.goals.back();
  h.ctx.set_nav_status(NavStatus::FAILED);
  h.bt.tick(h.ctx, 1.0);
  h.ctx.set_nav_status(NavStatus::MOVING);
  h.bt.tick(h.ctx, 2.0);
  EXPECT_GE(h.goals.size(), 2u);
  EXPECT_NE(h.goals.back().x, first.x);
}

// =============================================================================
//  Patrol tactical variants
// =============================================================================

TEST(BtDecision, PatrolUsesAggressiveRouteWhenOutpostAlive)
{
  Profile p = make_profile();
  p.patrol_aggressive = {{8.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.update(outpost_hp(500));  // outpost alive
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::PATROL);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 8.0);
}

TEST(BtDecision, PatrolFallsBackToDefaultWhenOutpostDestroyed)
{
  Profile p = make_profile();
  p.patrol = {{1.0, 1.0, 5.0}};
  p.patrol_aggressive = {{8.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.update(outpost_hp(0));  // outpost destroyed
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::PATROL);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);  // default route
}

TEST(BtDecision, PatrolResetsTrackingOnRouteSwitch)
{
  Profile p = make_profile();
  p.patrol = {{1.0, 1.0, 5.0}, {2.0, 2.0, 5.0}, {3.0, 3.0, 5.0}};
  p.patrol_aggressive = {{8.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.update(outpost_hp(500));  // outpost alive → aggressive
  h.bt.tick(h.ctx, 0.0);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals.back().x, 8.0);  // aggressive point

  h.ctx.update(outpost_hp(0));
  h.ctx.set_nav_status(NavStatus::MOVING);
  h.bt.tick(h.ctx, 1.0);
  EXPECT_DOUBLE_EQ(h.goals.back().x, 1.0)
      << "route switch must reset tracking and republish from new route start";
}

// =============================================================================
//  Resupply — enter / exit hysteresis
// =============================================================================

TEST(BtDecision, ResupplyExitsOnlyWhenFullyRecovered)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));  // hp low → enter
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::RESUPPLY);
  h.ctx.update(robot(FULL_HP, FULL_AMMO));  // hp == max AND ammo ok
  h.bt.tick(h.ctx, 1.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BtDecision, ResupplyStaysUntilHpFull)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::RESUPPLY);
  h.ctx.update(robot(399, FULL_AMMO));  // one below max → still healing
  h.bt.tick(h.ctx, 1.0);
  EXPECT_EQ(h.bt.state(), State::RESUPPLY);
}

TEST(BtDecision, ResupplyStaysUntilAmmoOk)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, 40));  // ammo low → enter
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::RESUPPLY);
  h.ctx.update(robot(FULL_HP, 99));  // ammo below ammo_ok(100)
  h.bt.tick(h.ctx, 1.0);
  EXPECT_EQ(h.bt.state(), State::RESUPPLY);
  h.ctx.update(robot(FULL_HP, 100));  // ammo == ammo_ok → leave
  h.bt.tick(h.ctx, 2.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BtDecision, ResupplySendsGoalToSupplyPad)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::RESUPPLY);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);
  EXPECT_DOUBLE_EQ(h.goals[0].y, -5.0);
}

TEST(BtDecision, ResupplyStaysPutOnPositionArrivalWithoutRfid)
{
  Profile p = make_profile();
  p.thresholds.resupply_timeout_s = 5.0;
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::RESUPPLY);
  const std::size_t goals_after_first = h.goals.size();

  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.bt.tick(h.ctx, 1.0);

  h.bt.tick(h.ctx, 10.0);
  h.bt.tick(h.ctx, 20.0);
  EXPECT_EQ(h.goals.size(), goals_after_first)
      << "after position arrival the sentry must stay put, not keep re-issuing "
         "supply goals every timeout";
}

TEST(BtDecision, BackupSupplyRotatedAfterTimeout)
{
  Profile p = make_profile();
  p.backup_supply_points = {{-3.0, -5.0, 0.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);   // primary
  h.bt.tick(h.ctx, 31.0);  // > resupply_timeout_s (30) → rotate
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, -3.0);   // backup
}

TEST(BtDecision, SupplyTargetCyclesBackToPrimary)
{
  Profile p = make_profile();
  p.backup_supply_points = {{-3.0, -5.0, 0.0}};
  p.thresholds.resupply_timeout_s = 1.0;
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);   // primary
  h.bt.tick(h.ctx, 2.0);
  EXPECT_DOUBLE_EQ(h.goals.back().x, -3.0);   // backup
  h.bt.tick(h.ctx, 4.0);
  EXPECT_DOUBLE_EQ(h.goals.back().x, -1.0);   // back to primary
}

TEST(BtDecision, ResupplyRotatesImmediatelyOnNavFail)
{
  Profile p = make_profile();
  p.backup_supply_points = {{-3.0, -5.0, 0.0}};
  Harness h{std::move(p)};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  h.ctx.set_nav_status(NavStatus::FAILED);
  h.bt.tick(h.ctx, 1.0);
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, -3.0);
}

// =============================================================================
//  Respawn — sentry must keep heading home after reviving (never gives up)
// =============================================================================

TEST(BtDecision, RespawnKeepsNavigatingHome)
{
  auto step = [](Harness & h, double t, uint16_t hp) {
    h.ctx.set_now(t);
    h.ctx.update(game(RUNNING, 300));
    h.ctx.update(robot(hp, FULL_AMMO));
    h.bt.tick(h.ctx, t);
  };

  Harness h{make_profile()};
  step(h, 0.0, 0);   // dead
  ASSERT_EQ(h.bt.state(), State::RESUPPLY);
  const std::size_t goals_before = h.goals.size();

  step(h, 200.0, 0);

  step(h, 201.0, 40);
  EXPECT_EQ(h.bt.state(), State::RESUPPLY);

  step(h, 240.0, 40);
  EXPECT_GT(h.goals.size(), goals_before)
      << "sentry must keep publishing supply goals after respawn, not stall";
}

// =============================================================================
//  Boundary cases
// =============================================================================

TEST(BtDecision, HpBoundaryNotEnterResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(150, FULL_AMMO));  // == hp_low (not <)
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BtDecision, HpBoundaryEnterResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(149, FULL_AMMO));  // < hp_low
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::RESUPPLY);
}

TEST(BtDecision, AmmoBoundaryEnterResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, 51));  // > ammo_low → still patrol
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BtDecision, IdleCancelsNavigation)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_FALSE(h.nav_cancelled);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::PREPARATION, 400));
  h.bt.tick(h.ctx, 1.0);
  EXPECT_TRUE(h.nav_cancelled);
}

TEST(BtDecision, MidPatrolHpDropGoesResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::PATROL);
  h.ctx.update(robot(140, FULL_AMMO));  // hp drops below hp_low
  h.bt.tick(h.ctx, 1.0);
  EXPECT_EQ(h.bt.state(), State::RESUPPLY);
}

// =============================================================================
//  Opening strike (kill enemy outpost at match start, once)
// =============================================================================

namespace
{
Profile strike_profile()
{
  Profile p = make_profile();
  p.opening_strike = {5.0, 0.0, 0.0};
  p.has_opening_strike = true;
  p.thresholds.opening_strike_duration_s = 90.0;
  return p;
}
}  // namespace

TEST(BtDecision, OpeningStrikeAtMatchStart)
{
  Harness h{strike_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::OPENING_STRIKE);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 5.0);
}

TEST(BtDecision, OpeningStrikeEndsAfterDwellThenPatrols)
{
  Harness h{strike_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::OPENING_STRIKE);
  h.bt.tick(h.ctx, 89.0);
  EXPECT_EQ(h.bt.state(), State::OPENING_STRIKE);
  h.bt.tick(h.ctx, 90.0);
  h.bt.tick(h.ctx, 91.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BtDecision, OpeningStrikeOnlyOncePerMatch)
{
  Harness h{strike_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::OPENING_STRIKE);
  h.bt.tick(h.ctx, 91.0);   // dwell done
  h.bt.tick(h.ctx, 92.0);
  ASSERT_EQ(h.bt.state(), State::PATROL);
  h.bt.tick(h.ctx, 200.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BtDecision, ResupplyPreemptsOpeningStrikeAndConsumesIt)
{
  Harness h{strike_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::OPENING_STRIKE);

  h.ctx.update(robot(100, FULL_AMMO));
  h.bt.tick(h.ctx, 5.0);
  ASSERT_EQ(h.bt.state(), State::RESUPPLY);

  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 6.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BtDecision, NoOpeningStrikeWhenNotConfigured)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 420));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
