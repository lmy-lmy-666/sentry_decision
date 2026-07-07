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

#include <gtest/gtest.h>

#include <vector>

#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "sentry_decision/context.hpp"
#include "sentry_decision/profile.hpp"
#include "sentry_decision/types.hpp"

using sentry_decision::CombatSubState;
using sentry_decision::Context;
using sentry_decision::DecisionFsm;
using sentry_decision::Profile;
using sentry_decision::State;
using sentry_decision::Waypoint;

namespace
{

Profile make_profile()
{
  Profile p;
  p.name = "test";
  p.thresholds.hp_low = 150;
  p.thresholds.hp_critical = 60;
  p.thresholds.heat_max = 240;
  p.thresholds.ammo_min = 1;
  p.thresholds.game_total_time = 420;
  p.thresholds.min_ticks_in_state = 1;
  p.thresholds.hp_low_exit_hysteresis = 180;
  p.thresholds.hp_critical_exit = 120;
  p.thresholds.resupply_timeout_s = 60.0;
  p.thresholds.enemy_lost_time_s = 5.0;
  p.thresholds.engage_distance = 3.0;
  p.thresholds.track_distance = 8.0;
  p.patrol = {{1.0, 1.0, 5.0}, {2.0, 2.0, 5.0}};
  p.attack_push = {{10.0, 0.0, 5.0}};
  p.supply = {-1.0, -5.0, 0.0};
  p.retreat = {-2.0, -5.0, 0.0};
  p.safe_cover = {2.0, 0.0, 0.0};
  p.enable_attack_push = true;
  return p;
}

rm_interfaces::msg::GameStatus game(uint8_t progress, int32_t remain)
{
  rm_interfaces::msg::GameStatus m;
  m.game_progress = progress;
  m.stage_remain_time = remain;
  return m;
}

rm_interfaces::msg::RobotStatus robot(uint16_t hp, uint16_t ammo, uint16_t heat = 0)
{
  rm_interfaces::msg::RobotStatus m;
  m.current_hp = hp;
  m.maximum_hp = 400;
  m.projectile_allowance_17mm = ammo;
  m.shooter_17mm_1_barrel_heat = heat;
  return m;
}

rm_interfaces::msg::GameRobotHP outpost(uint16_t hp)
{
  rm_interfaces::msg::GameRobotHP m;
  m.ally_outpost_hp = hp;
  return m;
}

struct Harness
{
  std::vector<Waypoint> goals;
  std::vector<sentry_decision::StanceCommand> stances;
  bool nav_cancelled{false};
  Context ctx;
  DecisionFsm fsm;

  explicit Harness(Profile p)
  : fsm(
      std::move(p), [this](const Waypoint & w) { goals.push_back(w); },
      [this](sentry_decision::StanceCommand s) { stances.push_back(s); },
      [this]() { nav_cancelled = true; })
  {
    ctx.set_thresholds(make_profile().thresholds);
    ctx.set_now(0.0);
  }
};

constexpr uint8_t RUNNING = rm_interfaces::msg::GameStatus::RUNNING;

}  // namespace

TEST(DecisionFsm, IdleWhenGameNotRunning)
{
  Harness h{make_profile()};
  h.ctx.update(game(rm_interfaces::msg::GameStatus::PREPARATION, 400));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
  EXPECT_TRUE(h.goals.empty());
}

TEST(DecisionFsm, PatrolWhenHealthyOutpostDead)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);
}

TEST(DecisionFsm, AttackPushWhenOutpostAlive)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(500));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::ATTACK_PUSH);
}

TEST(DecisionFsm, ResupplyWhenAmmoEmpty)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 0));
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

TEST(DecisionFsm, GuardPreemptsPatrolMidRoute)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  h.ctx.update(robot(30, 50));
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RETREAT);
}

TEST(DecisionFsm, CriticalHpBeatsResupply)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(30, 0));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RETREAT);
}

TEST(DecisionFsm, PatrolRouteAdvancesAndCycles)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.fsm.tick(h.ctx, 0.0);
  h.ctx.set_nav_status(sentry_decision::NavStatus::ARRIVED);
  h.fsm.tick(h.ctx, 1.0);
  h.fsm.tick(h.ctx, 6.0);
  h.ctx.set_nav_status(sentry_decision::NavStatus::MOVING);
  h.fsm.tick(h.ctx, 6.0);
  ASSERT_GE(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, 1.0);
  EXPECT_DOUBLE_EQ(h.goals[1].x, 2.0);
}

TEST(DecisionFsm, PatrolDoesNotAdvanceBeforeArrival)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.fsm.tick(h.ctx, 0.0);
  h.fsm.tick(h.ctx, 100.0);
  EXPECT_EQ(h.goals.size(), 1u);
}

TEST(DecisionFsm, PatrolDwellStartsAfterArrival)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.fsm.tick(h.ctx, 0.0);
  h.ctx.set_nav_status(sentry_decision::NavStatus::ARRIVED);
  h.fsm.tick(h.ctx, 10.0);
  h.fsm.tick(h.ctx, 14.0);
  EXPECT_EQ(h.goals.size(), 1u);
  h.fsm.tick(h.ctx, 15.0);
  h.ctx.set_nav_status(sentry_decision::NavStatus::MOVING);
  h.fsm.tick(h.ctx, 15.0);
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, 2.0);
}

TEST(DecisionFsm, RefereeStaleForcesIdle)
{
  Harness h{make_profile()};
  h.ctx.set_now(0.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(500));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::ATTACK_PUSH);
  h.ctx.set_now(4.0);
  h.fsm.tick(h.ctx, 4.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

TEST(DecisionFsm, AttackPushDisabledInLateGame)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 30));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(500));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, AttackPushRequiresHealthyHp)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(170, 50));
  h.ctx.update(outpost(500));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

TEST(DecisionFsm, BackupSupplyPublishesAfterTimeout)
{
  Profile p = make_profile();
  p.backup_supply_points = {{-3.0, -5.0, 0.0}};
  Harness h{p};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.goals.size(), 1u);
  h.fsm.tick(h.ctx, 61.0);
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, -3.0);
}

TEST(DecisionFsm, EnemyDetectedPreemptsAttackPush)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(500));
  sentry_decision::EnemyInfo enemy;
  enemy.detected = true;
  enemy.nearest_distance = 6.0;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
}

TEST(DecisionFsm, EnemyDistanceDrivesCombatSubstate)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  sentry_decision::EnemyInfo enemy;
  enemy.detected = true;
  enemy.nearest_distance = 2.0;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::TRACK);
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::ENGAGE);
}

TEST(DecisionFsm, AerialThreatSendsEnhancedDefenseImmediately)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  sentry_decision::EnemyInfo enemy;
  enemy.aerial_threat = true;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::HARDEN);
  ASSERT_FALSE(h.stances.empty());
  EXPECT_EQ(h.stances[0], sentry_decision::StanceCommand::ENHANCED_DEFENSIVE);
}

TEST(DecisionFsm, DefensiveUpgradeBypassesStanceCooldown)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.stances.size(), 1u);
  EXPECT_EQ(h.stances[0], sentry_decision::StanceCommand::MOBILITY);
  sentry_decision::EnemyInfo enemy;
  enemy.aerial_threat = true;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 1.0);
  ASSERT_EQ(h.stances.size(), 2u);
  EXPECT_EQ(h.stances[1], sentry_decision::StanceCommand::ENHANCED_DEFENSIVE);
}

TEST(DecisionFsm, IdleWhenGameEnds)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(500));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::ATTACK_PUSH);
  h.ctx.update(game(rm_interfaces::msg::GameStatus::GAME_OVER, 0));
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::IDLE);
}

// =========================================================================
// 鲁棒性测试
// =========================================================================

TEST(DecisionFsm, AmmoZeroEnemyGoesResupply)
{
  // 没弹药时不管有没有敌人, 必须去补给
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 0));  // ammo=0
  sentry_decision::EnemyInfo enemy;
  enemy.detected = true;
  enemy.nearest_distance = 3.0;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, AmmoZeroInResupplyEnemyDoesNotBreak)
{
  // 没弹药时在补给途中, 敌人出现不退出
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 0));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  sentry_decision::EnemyInfo enemy;
  enemy.detected = true;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, NavStuckAdvancesPatrol)
{
  // 导航卡住 → 跳下一个巡逻点
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));  // 前哨站死 → PATROL
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  ASSERT_FALSE(h.goals.empty());
  const auto first_goal = h.goals.back();
  h.ctx.set_nav_status(sentry_decision::NavStatus::STUCK);
  h.fsm.tick(h.ctx, 1.0);
  h.ctx.set_nav_status(sentry_decision::NavStatus::MOVING);
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_GE(h.goals.size(), 2u);
  EXPECT_NE(h.goals.back().x, first_goal.x);
}

TEST(DecisionFsm, HpBoundaryNotEnterRetreat)
{
  // hp=60 刚好不触发 critical
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(60, 50));
  h.ctx.update(outpost(500));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_NE(h.fsm.state(), State::RETREAT);
}

TEST(DecisionFsm, HpBoundaryEnterRetreat)
{
  // hp=59 触发 critical
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(59, 50));
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::RETREAT);
}

TEST(DecisionFsm, EnemyAppearsMidResupplyGoesDefend)
{
  // 有弹药时补给途中敌人出现 → 去战斗
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));  // hp<150 触发 RESUPPLY
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  sentry_decision::EnemyInfo enemy;
  enemy.detected = true;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
}

TEST(DecisionFsm, EnemyGoneReturnsToResupply)
{
  // 战斗完敌人消失 → 血量还低 → 回 RESUPPLY
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));
  sentry_decision::EnemyInfo enemy;
  enemy.detected = true;
  enemy.nearest_distance = 2.0;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  enemy.detected = false;
  h.ctx.update(enemy);
  h.ctx.set_now(1.0);
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, EngagesEnemyAtCloseRange)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  sentry_decision::EnemyInfo enemy;
  enemy.detected = true;
  enemy.nearest_distance = 1.5;  // <3m → ENGAGE
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::TRACK);
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::ENGAGE);
}

TEST(DecisionFsm, RetreatSendSupplyNotRetreat)
{
  // RETREAT 发 supply 点, 不是 retreat 点
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(40, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RETREAT);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, -1.0);  // supply.x
  EXPECT_DOUBLE_EQ(h.goals[0].y, -5.0);  // supply.y
}

TEST(DecisionFsm, ResupplyExitAtFullHp)
{
  // RESUPPLY 保持到满血
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));  // hp<150
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  h.ctx.update(robot(399, 50));
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 2.0);
  EXPECT_NE(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, DefendExitWhenNoThreat)
{
  // 没有威胁 → 退出 DEFEND
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));  // 前哨站死 → PATROL
  sentry_decision::EnemyInfo enemy;
  enemy.detected = true;
  enemy.nearest_distance = 2.0;
  h.ctx.update(enemy);
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  enemy.detected = false;
  h.ctx.update(enemy);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.fsm.tick(h.ctx, 5.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
