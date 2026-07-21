// Copyright 2026 Boombroke
//
// Unit tests for BUMP_TRAVERSE — crossing undulating (washboard) terrain
// open-loop, bypassing Nav2. Swerve chassis: pure linear.x, CONSTANT speed
// (no slowdown), y/yaw = 0. Position-aware auto-trigger.
//
// Same behavioural assertions as the FSM sample's bump_test.cpp — the BT port's
// 5-phase subtree (GOTO_ENTRY→ALIGN→DASHING→DONE/FAILED) is behaviour-equal.
//
#include "omni_behavior_sample/bt_decision.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "omni_behavior_sample/context.hpp"
#include "omni_behavior_sample/profile.hpp"
#include "omni_behavior_sample/types.hpp"

using omni_behavior_sample::BumpSegment;
using omni_behavior_sample::BtDecision;
using omni_behavior_sample::Context;
using omni_behavior_sample::NavStatus;
using omni_behavior_sample::Profile;
using omni_behavior_sample::State;
using omni_behavior_sample::Waypoint;

namespace
{

constexpr uint8_t RUNNING = rm_interfaces::msg::GameStatus::RUNNING;
constexpr uint16_t FULL_HP = 400;
constexpr uint16_t FULL_AMMO = 300;

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

// A profile with one X-aligned bump segment from x=1 to x=5 at y=0.
// Patrol goal sits on the far side (x=8) so a robot at x=0 must cross.
Profile bump_profile()
{
  Profile p;
  p.name = "bump_test";
  p.thresholds.min_ticks_in_state = 1;   // fast tests
  p.thresholds.bump_dash_speed = 1.0;
  p.thresholds.bump_reverse_speed = 0.5;
  p.thresholds.bump_tol = 0.25;
  p.thresholds.bump_entry_radius = 0.5;
  p.thresholds.bump_align_time_s = 0.5;
  p.thresholds.bump_timeout_s = 20.0;
  p.patrol = {{8.0, 0.0, 5.0}};           // far side of the segment
  p.supply = {-1.0, 0.0, 0.0};            // near side (retreat target)
  BumpSegment seg;
  seg.entry = {1.0, 0.0, 0.0};
  seg.exit  = {5.0, 0.0, 0.0};
  seg.yaw = 0.0;
  p.bump_segments = {seg};
  return p;
}

struct Harness
{
  std::vector<Waypoint> goals;
  std::vector<double>   vels;       // every bump velocity command (in order)
  bool nav_cancelled{false};
  Context ctx;
  BtDecision bt;

  explicit Harness(Profile p)
  : bt(
      std::move(p),
      [this](const Waypoint & w) { goals.push_back(w); },
      [this]() { nav_cancelled = true; },
      [this](double vx) { vels.push_back(vx); })
  {
    ctx.set_thresholds(bump_profile().thresholds);
    ctx.set_now(0.0);
  }

  void healthy_at(double x, double y, double t)
  {
    ctx.set_now(t);
    ctx.update(game(RUNNING, 300));
    ctx.update(robot(FULL_HP, FULL_AMMO));
    ctx.set_sentry_position(x, y);
  }

  double last_vel() const { return vels.empty() ? 0.0 : vels.back(); }
};

}  // namespace


TEST(BumpTraverse, EntersWhenSegmentBetweenRobotAndGoal)
{
  Harness h{bump_profile()};
  h.healthy_at(0.0, 0.0, 0.0);
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::BUMP_TRAVERSE);
}

TEST(BumpTraverse, NoTriggerWhenGoalOnSameSide)
{
  Profile p = bump_profile();
  p.patrol = {{-2.0, 0.0, 5.0}};
  Harness h{std::move(p)};
  h.healthy_at(0.0, 0.0, 0.0);
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BumpTraverse, NoTriggerWhenOutsideYCorridor)
{
  Harness h{bump_profile()};
  h.healthy_at(0.0, 5.0, 0.0);
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

TEST(BumpTraverse, NoTriggerWhenNoSegmentsConfigured)
{
  Profile p = bump_profile();
  p.bump_segments.clear();
  Harness h{std::move(p)};
  h.healthy_at(0.0, 0.0, 0.0);
  h.bt.tick(h.ctx, 0.0);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}


// Full flow: GOTO_ENTRY -> ALIGN -> DASHING(constant) -> DONE -> PATROL
TEST(BumpTraverse, FullFlowForwardCrossing)
{
  Harness h{bump_profile()};
  h.healthy_at(0.0, 0.0, 0.0);

  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::BUMP_TRAVERSE);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);

  h.healthy_at(1.0, 0.0, 0.5);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.bt.tick(h.ctx, 0.5);
  EXPECT_TRUE(h.nav_cancelled);

  h.bt.tick(h.ctx, 0.6);
  EXPECT_DOUBLE_EQ(h.last_vel(), 0.0);

  h.healthy_at(1.0, 0.0, 1.1);
  h.bt.tick(h.ctx, 1.1);
  EXPECT_DOUBLE_EQ(h.last_vel(), 1.0);

  h.healthy_at(3.0, 0.0, 2.0);
  h.bt.tick(h.ctx, 2.0);
  EXPECT_DOUBLE_EQ(h.last_vel(), 1.0);

  h.healthy_at(5.0, 0.0, 3.0);
  h.bt.tick(h.ctx, 3.0);
  EXPECT_DOUBLE_EQ(h.last_vel(), 0.0);

  for (double t = 3.1; t < 4.0; t += 0.1) h.bt.tick(h.ctx, t);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}


// Helper: drive the tree into DASHING and return the harness ready to dash.
static void drive_to_dashing(Harness & h)
{
  h.healthy_at(0.0, 0.0, 0.0);
  h.bt.tick(h.ctx, 0.0);                 // GOTO_ENTRY, goal sent
  h.healthy_at(1.0, 0.0, 0.5);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.bt.tick(h.ctx, 0.5);                 // -> ALIGN
  h.healthy_at(1.0, 0.0, 1.1);
  h.bt.tick(h.ctx, 1.1);                 // -> DASHING
}

TEST(BumpTraverse, DashingNotPreemptedByLowHp)
{
  Harness h{bump_profile()};
  drive_to_dashing(h);
  ASSERT_EQ(h.bt.state(), State::BUMP_TRAVERSE);
  h.ctx.set_now(2.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(50, FULL_AMMO));     // would normally force RESUPPLY
  h.ctx.set_sentry_position(3.0, 0.0);
  h.bt.tick(h.ctx, 2.0);
  EXPECT_EQ(h.bt.state(), State::BUMP_TRAVERSE)
      << "low hp must not interrupt an in-progress bump dash (stranded risk)";
  EXPECT_DOUBLE_EQ(h.last_vel(), 1.0);    // still dashing
}

TEST(BumpTraverse, DashingInterruptedByGameOver)
{
  Harness h{bump_profile()};
  drive_to_dashing(h);
  ASSERT_EQ(h.bt.state(), State::BUMP_TRAVERSE);
  h.ctx.set_now(2.0);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::GAME_OVER, 0));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.set_sentry_position(3.0, 0.0);
  h.bt.tick(h.ctx, 2.0);
  EXPECT_EQ(h.bt.state(), State::IDLE);
}

TEST(BumpTraverse, BlindDashHoldsLastVelocityOnPositionLoss)
{
  Harness h{bump_profile()};
  drive_to_dashing(h);
  ASSERT_DOUBLE_EQ(h.last_vel(), 1.0);
  h.ctx.set_now(2.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(FULL_HP, FULL_AMMO));
  h.ctx.invalidate_sentry_position();
  h.bt.tick(h.ctx, 2.0);
  EXPECT_DOUBLE_EQ(h.last_vel(), 1.0)
      << "on TF loss keep dashing at last velocity, never stop (trough-stall risk)";
  EXPECT_EQ(h.bt.state(), State::BUMP_TRAVERSE);
}

TEST(BumpTraverse, DashTimeoutTriggersReverseRecovery)
{
  Profile p = bump_profile();
  p.thresholds.bump_timeout_s = 5.0;
  Harness h{std::move(p)};
  drive_to_dashing(h);
  ASSERT_EQ(h.bt.state(), State::BUMP_TRAVERSE);
  h.healthy_at(3.0, 0.0, 20.0);
  h.bt.tick(h.ctx, 20.0);
  h.healthy_at(3.0, 0.0, 20.1);
  h.bt.tick(h.ctx, 20.1);
  EXPECT_LT(h.last_vel(), 0.0)
      << "after dash timeout the sentry must reverse-crawl back toward entry";
}

TEST(BumpTraverse, ConstantSpeedNoSlowdownNearExit)
{
  Harness h{bump_profile()};
  drive_to_dashing(h);
  h.healthy_at(2.0, 0.0, 2.0);
  h.bt.tick(h.ctx, 2.0);
  const double far = h.last_vel();
  h.healthy_at(4.5, 0.0, 2.5);
  h.bt.tick(h.ctx, 2.5);
  const double near = h.last_vel();
  EXPECT_DOUBLE_EQ(far, near)
      << "speed must stay constant near the exit — no ramp-down on washboard";
  EXPECT_DOUBLE_EQ(near, 1.0);
}

TEST(BumpTraverse, BackwardCrossingNegativeSpeed)
{
  Profile p = bump_profile();
  p.patrol = {{-2.0, 0.0, 5.0}};   // goal on low side
  Harness h{std::move(p)};
  h.healthy_at(8.0, 0.0, 0.0);
  h.bt.tick(h.ctx, 0.0);
  ASSERT_EQ(h.bt.state(), State::BUMP_TRAVERSE);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, 5.0);
  h.healthy_at(5.0, 0.0, 0.5);
  h.ctx.set_nav_status(NavStatus::ARRIVED);
  h.bt.tick(h.ctx, 0.5);
  h.healthy_at(5.0, 0.0, 1.1);
  h.bt.tick(h.ctx, 1.1);
  EXPECT_LT(h.last_vel(), 0.0);
  EXPECT_DOUBLE_EQ(h.last_vel(), -1.0);
}

TEST(BumpTraverse, DoneSettlesForConfiguredStopTicks)
{
  Profile p = bump_profile();
  p.thresholds.bump_stop_ticks = 3;
  Harness h{std::move(p)};
  drive_to_dashing(h);
  h.healthy_at(5.0, 0.0, 2.0);
  h.bt.tick(h.ctx, 2.0);
  ASSERT_EQ(h.bt.state(), State::BUMP_TRAVERSE);   // DONE but still settling
  h.healthy_at(5.0, 0.0, 2.1);
  h.bt.tick(h.ctx, 2.1);
  EXPECT_EQ(h.bt.state(), State::BUMP_TRAVERSE)
      << "must settle bump_stop_ticks frames before handing back";
  for (double t = 2.2; t < 3.0; t += 0.1) h.bt.tick(h.ctx, t);
  EXPECT_EQ(h.bt.state(), State::PATROL);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
