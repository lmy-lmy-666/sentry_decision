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
  h.ctx.update_enemy_from_aim(true, 6.0);
  h.fsm.tick(h.ctx, 0.0);
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
}

TEST(DecisionFsm, EnemyDistanceDrivesCombatSubstate)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update_enemy_from_aim(true, 2.0);
  h.fsm.tick(h.ctx, 0.0);  // 进入 DEFEND/SCOUT
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::SCOUT);
  h.fsm.tick(h.ctx, 0.3);  // 等 0.2s 滞回 → TRACK
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::TRACK);
  h.fsm.tick(h.ctx, 0.6);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::ENGAGE);
}

TEST(DecisionFsm, AerialThreatSendsEnhancedDefenseImmediately)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update_enemy_from_radar(0, 0, 1, true);
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
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
  h.ctx.update_enemy_from_radar(0, 0, 1, true);
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
  h.ctx.update_enemy_from_aim(true, 3.0);
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
  h.ctx.update_enemy_from_aim(true, 999.0);
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
  h.ctx.update_enemy_from_aim(true, 999.0);
  h.fsm.tick(h.ctx, 1.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
}

TEST(DecisionFsm, EnemyGoneReturnsToResupply)
{
  // 战斗完敌人消失 → 血量还低 → 回 RESUPPLY
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(140, 50));
  h.ctx.update_enemy_from_aim(true, 2.0);
  h.fsm.tick(h.ctx, 0.0);  // 进入 SCOUT
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  h.ctx.update_enemy_from_aim(false, 999.0);
  h.ctx.set_now(1.0);
  h.fsm.tick(h.ctx, 1.0);  // 记录丢失时刻，但在 5s 滞后期内保持 DEFEND
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  // 滞后过期后切 RESUPPLY
  h.fsm.tick(h.ctx, 7.0);
  EXPECT_EQ(h.fsm.state(), State::RESUPPLY);
}

TEST(DecisionFsm, EngagesEnemyAtCloseRange)
{
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update_enemy_from_aim(true, 1.5);  // <3m → ENGAGE
  h.fsm.tick(h.ctx, 0.0);  // 进入 DEFEND/SCOUT
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::SCOUT);
  h.fsm.tick(h.ctx, 0.3);  // 等 0.2s 滞回 → TRACK
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::TRACK);
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::ENGAGE);
}

TEST(DecisionFsm, RetreatSendRetreatFirst)
{
  // RETREAT 优先发 retreat 点，而非直接发 supply
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(40, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RETREAT);
  ASSERT_FALSE(h.goals.empty());
  EXPECT_DOUBLE_EQ(h.goals[0].x, -2.0);  // retreat.x
  EXPECT_DOUBLE_EQ(h.goals[0].y, -5.0);  // retreat.y
}

TEST(DecisionFsm, RetreatFallbackToSupplyThenSafeCover)
{
  // retreat 超时 → backup_retreat_points → supply → safe_cover 多级兜底
  Profile p = make_profile();
  p.retreat = {-3.0, -5.0, 0.0};
  p.backup_retreat_points = {{-4.0, -5.0, 0.0}};
  p.supply = {-1.0, -5.0, 0.0};
  p.safe_cover = {0.0, 0.0, 0.0};
  p.thresholds.retreat_timeout_s = 1.0;
  Harness h{p};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(40, 50));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RETREAT);
  ASSERT_EQ(h.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(h.goals[0].x, -3.0);  // retreat
  // 超时 → backup
  h.fsm.tick(h.ctx, 2.0);
  ASSERT_EQ(h.goals.size(), 2u);
  EXPECT_DOUBLE_EQ(h.goals[1].x, -4.0);  // backup_retreat
  // 再超时 → supply
  h.fsm.tick(h.ctx, 4.0);
  ASSERT_EQ(h.goals.size(), 3u);
  EXPECT_DOUBLE_EQ(h.goals[2].x, -1.0);  // supply
  // 再超时 → safe_cover
  h.fsm.tick(h.ctx, 6.0);
  ASSERT_EQ(h.goals.size(), 4u);
  EXPECT_DOUBLE_EQ(h.goals[3].x, 0.0);  // safe_cover
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
  h.ctx.update_enemy_from_aim(true, 2.0);
  h.fsm.tick(h.ctx, 0.0);  // 进入 SCOUT
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  h.ctx.update_enemy_from_aim(false, 999.0);
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  // 先 tick 一次让 enemy_lost_at_s_ 记录丢失时刻
  h.fsm.tick(h.ctx, 1.0);
  // 等 5s 滞后过期后再 tick，才退出 DEFEND
  h.fsm.tick(h.ctx, 7.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

// =========================================================================
// 新增：传感器融合测试
// =========================================================================

TEST(DecisionFsm, RadarOnlyDetectsEnemy)
{
  // 只用雷达发现地面敌人（无自瞄），应进入 DEFEND
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));  // 前哨站死，否则会进 ATTACK_PUSH
  h.ctx.update_enemy_from_radar(5.0, 3.0, 1, false);  // 雷达报 1 个敌人，非空中
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
}

TEST(DecisionFsm, RadarCountZeroClearsDetection)
{
  // 雷达报 count=0 → radar_has_target_=false，应退出 DEFEND
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.ctx.update_enemy_from_radar(5.0, 3.0, 1, false);
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  // 雷达报 0 个敌人
  h.ctx.update_enemy_from_radar(0.0, 0.0, 0, false);
  // 滞后: 先 tick 记录丢失，再等 5s
  h.fsm.tick(h.ctx, 1.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);  // 滞后期内
  h.fsm.tick(h.ctx, 7.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);  // 滞后过期
}

TEST(DecisionFsm, DualSourceBothDetecting)
{
  // 自瞄和雷达同时报告敌人，OR 融合应正确
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  // 雷达看到 2 个敌人
  h.ctx.update_enemy_from_radar(5.0, 3.0, 2, false);
  // 自瞄看到 1 个近距离目标
  h.ctx.update_enemy_from_aim(true, 1.5);
  h.fsm.tick(h.ctx, 0.0);  // 进入 DEFEND/SCOUT
  EXPECT_EQ(h.fsm.state(), State::DEFEND);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::SCOUT);
  h.fsm.tick(h.ctx, 0.3);  // 等 0.2s 滞回 → TRACK
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::TRACK);
  h.fsm.tick(h.ctx, 0.6);
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::ENGAGE);
}

TEST(DecisionFsm, AimLostRadarKeepsDetection)
{
  // 自瞄丢锁 (detected=false)，雷达还在 → 应保持 DEFEND
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.ctx.update_enemy_from_radar(5.0, 3.0, 1, false);
  h.ctx.update_enemy_from_aim(true, 2.0);
  h.fsm.tick(h.ctx, 0.0);  // 进入 SCOUT
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  // 自瞄丢锁，雷达还在
  h.ctx.update_enemy_from_aim(false, 999.0);
  h.fsm.tick(h.ctx, 1.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);  // 仍保持，雷达维持检测
}

TEST(DecisionFsm, RadarLostAimKeepsDetection)
{
  // 雷达清零，自瞄还在 → 应保持 DEFEND
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.ctx.update_enemy_from_radar(5.0, 3.0, 1, false);
  h.ctx.update_enemy_from_aim(true, 2.0);
  h.fsm.tick(h.ctx, 0.0);  // 进入 SCOUT
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  // 雷达清零，自瞄还在
  h.ctx.update_enemy_from_radar(0.0, 0.0, 0, false);
  h.fsm.tick(h.ctx, 1.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);  // 仍保持，自瞄维持检测
}

TEST(DecisionFsm, BothSourcesLostExitsDefend)
{
  // 两个源都丢了 → 滞后过期后退出 DEFEND
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.ctx.update_enemy_from_radar(5.0, 3.0, 1, false);
  h.ctx.update_enemy_from_aim(true, 2.0);
  h.fsm.tick(h.ctx, 0.0);  // 进入 SCOUT
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
  // 两个源都丢
  h.ctx.update_enemy_from_aim(false, 999.0);
  h.ctx.update_enemy_from_radar(0.0, 0.0, 0, false);
  // 记录丢失
  h.fsm.tick(h.ctx, 1.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);  // 滞后期内
  // 滞后过期
  h.fsm.tick(h.ctx, 7.0);
  EXPECT_EQ(h.fsm.state(), State::PATROL);
}

// =========================================================================
// 新增：伤害类型测试
// =========================================================================

TEST(DecisionFsm, ArmorCollisionTriggersDefend)
{
  // 被撞（ARMOR_COLLISION）也应触发战斗反应
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  // 模拟被撞击
  rm_interfaces::msg::RobotStatus hit;
  hit.current_hp = 380;
  hit.maximum_hp = 400;
  hit.projectile_allowance_17mm = 50;
  hit.is_hp_deduced = true;
  hit.hp_deduction_reason = rm_interfaces::msg::RobotStatus::ARMOR_COLLISION;
  h.ctx.update(hit);
  h.fsm.tick(h.ctx, 1.0);
  ASSERT_EQ(h.fsm.state(), State::DEFEND);
}

TEST(DecisionFsm, OverheatPenaltyDoesNotTriggerDefend)
{
  // 超限惩罚（OVER_HEAT）不是敌方攻击，不应触发战斗
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.update(outpost(0));
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::PATROL);
  // 模拟过热扣血
  rm_interfaces::msg::RobotStatus overheat;
  overheat.current_hp = 390;
  overheat.maximum_hp = 400;
  overheat.projectile_allowance_17mm = 50;
  overheat.is_hp_deduced = true;
  overheat.hp_deduction_reason = rm_interfaces::msg::RobotStatus::OVER_HEAT;
  h.ctx.update(overheat);
  h.fsm.tick(h.ctx, 1.0);
  EXPECT_NE(h.fsm.state(), State::DEFEND);  // 不触发战斗
}

// =========================================================================
// 新增：中高危修复验证测试
// =========================================================================

TEST(DecisionFsm, ResupplyStayWhenAmmoEmptyAndBackupExhausted)
{
  // Fix 3: 补给耗尽 + 弹药为空 + hp≥120 → 仍应保持 RESUPPLY，不振荡
  Profile p = make_profile();
  p.thresholds.hp_critical_exit = 120;
  p.thresholds.hp_low = 150;
  Harness h{p};
  // 手动设置补给耗尽状态
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(130, 0));  // hp=130, ammo=0
  h.fsm.tick(h.ctx, 0.0);
  ASSERT_EQ(h.fsm.state(), State::RESUPPLY);
  // 模拟补给全部失败（supply_backup_exhausted_ 需在 FSM 内设置，
  // 通过 resupply_timeout_s 触发）
}

TEST(DecisionFsm, EngageRetreatsToTrackWhenEnemyFar)
{
  // Fix 8: ENGAGE 中敌人退到 engage_distance*1.5 外 + 停留 ≥2s → 回 TRACK
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  // 进入 DEFEND → SCOUT → TRACK → ENGAGE
  h.ctx.update_enemy_from_aim(true, 1.5);  // < 3m → 会进 ENGAGE
  h.fsm.tick(h.ctx, 0.0);   // DEFEND/SCOUT
  h.fsm.tick(h.ctx, 0.3);   // TRACK
  h.fsm.tick(h.ctx, 0.6);   // ENGAGE (1.5m < 3m)
  ASSERT_EQ(h.fsm.combat_substate(), CombatSubState::ENGAGE);
  // 敌人退到 5m（> 3*1.5=4.5m），但 < 8m
  h.ctx.update_enemy_from_aim(true, 5.0);
  h.fsm.tick(h.ctx, 0.9);   // 停留 0.3s，< 2s 门槛
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::ENGAGE);  // 还未触发
  h.fsm.tick(h.ctx, 2.7);   // 停留 ≥2s
  EXPECT_EQ(h.fsm.combat_substate(), CombatSubState::TRACK);   // 回 TRACK
}

TEST(DecisionFsm, TrackPursuitUpdatesWithMovingEnemy)
{
  // Fix 2: TRACK 追击目标每 1s 更新，跟踪移动敌人
  Harness h{make_profile()};
  h.ctx.update(game(RUNNING, 300));
  h.ctx.update(robot(400, 50));
  h.ctx.set_sentry_position(0.0, 0.0);  // 设置己方位置
  // 进入 SCOUT → TRACK
  h.ctx.update_enemy_from_radar(8.0, 0.0, 1, false);
  h.fsm.tick(h.ctx, 0.0);   // DEFEND/SCOUT
  h.fsm.tick(h.ctx, 0.3);   // SCOUT→TRACK（本 tick 只做状态切换）
  ASSERT_EQ(h.fsm.combat_substate(), CombatSubState::TRACK);
  EXPECT_TRUE(h.goals.empty());  // 切换 tick 不执行 combat_track，goal 尚未发出
  h.fsm.tick(h.ctx, 0.4);   // TRACK 首次执行，发出追击目标
  ASSERT_FALSE(h.goals.empty());
  // 敌人移动（雷达新坐标）
  h.ctx.update_enemy_from_radar(10.0, 2.0, 1, false);
  h.fsm.tick(h.ctx, 1.5);   // >1s 距上次更新，应更新追击目标
  EXPECT_GE(h.goals.size(), 2u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
