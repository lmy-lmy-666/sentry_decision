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

#include <utility>

namespace sentry_decision
{

DecisionFsm::DecisionFsm(
  Profile profile, GoalPublisher goal_pub, StanceSender stance_send, NavCanceller nav_cancel)
: profile_(std::move(profile)),
  publish_goal_(std::move(goal_pub)),
  send_stance_(std::move(stance_send)),
  cancel_nav_(std::move(nav_cancel))
{
}

void DecisionFsm::tick(const Context & ctx, double now_s)
{
  // 敌人丢失计时——必须在 select_state 之前更新，否则读到的永远是旧值
  if (state_ == State::DEFEND) {
    if (ctx.enemy_detected() || ctx.under_attack() || ctx.under_aerial_attack()) {
      enemy_lost_at_s_ = 0.0;
    } else if (enemy_lost_at_s_ == 0.0) {
      enemy_lost_at_s_ = now_s;
    }
  }

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

State DecisionFsm::select_state(const Context & ctx, double now_s) const
{
  if (!ctx.referee_fresh() || !ctx.game_running()) {
    return State::IDLE;
  }

  if (state_ == State::RETREAT) {
    if (ctx.hp() < profile_.thresholds.hp_critical_exit) return State::RETREAT;
  } else if (ctx.hp_critical()) {
    return State::RETREAT;
  }

  if (ctx.under_aerial_attack()) {
    return State::DEFEND;
  }

  if (state_ == State::RESUPPLY) {
    // 没弹药时不管有没有敌人都不退出补给（没法反击）
    // 有弹药但 hp 低时，敌人出现才让路给 DEFEND
    if (ctx.ammo_empty() || !ctx.enemy_detected()) {
      if (supply_backup_exhausted_) {
        if (ctx.hp() < profile_.thresholds.hp_critical_exit || ctx.ammo_empty()) {
          return State::RESUPPLY;
        }
      } else if (ctx.ammo_empty() || ctx.hp() < ctx.max_hp()) {
        // 补给区不好走，到了就补满再出发
        return State::RESUPPLY;
      }
    }
  } else if (ctx.needs_resupply()) {
    // supply_backup_exhausted_ 为 true 表示上一轮补给已耗尽所有
    // 可选点仍未到达。此时除非弹药耗尽或冷却时间已过，否则不重试
    // RESUPPLY，避免 PATROL↔RESUPPLY 死循环振荡。
    bool supply_in_cooldown = supply_backup_exhausted_ && !ctx.ammo_empty() &&
      (supply_fail_time_ > 0.0) &&
      (now_s - supply_fail_time_ < profile_.thresholds.supply_retry_cooldown_s);
    if (!supply_in_cooldown && (ctx.ammo_empty() || !ctx.enemy_detected())) {
      // 敌人刚消失时暂不切 RESUPPLY——遵守与 DEFEND 相同的滞后延迟
      if (state_ == State::DEFEND && enemy_lost_at_s_ > 0.0 &&
          now_s - enemy_lost_at_s_ < profile_.thresholds.enemy_lost_time_s) {
        // 还在滞后期内，保持 DEFEND，不切 RESUPPLY
      } else {
        return State::RESUPPLY;
      }
    }
  }

  if (ctx.enemy_detected() || ctx.under_attack()) {
    return State::DEFEND;
  }

  // 敌人刚消失时暂不退出 DEFEND——滞后 enemy_lost_time_s(默认5s)防止振荡
  if (state_ == State::DEFEND && enemy_lost_at_s_ > 0.0 &&
      now_s - enemy_lost_at_s_ < profile_.thresholds.enemy_lost_time_s) {
    return State::DEFEND;  // 还在滞后期内，保持 DEFEND
  }

  if (attack_push_allowed(ctx)) {
    return State::ATTACK_PUSH;
  }

  return State::PATROL;
}

bool DecisionFsm::can_leave_current_state(State next) const
{
  if (next == state_) return true;
  if (state_ == State::IDLE) return true;
  if (next == State::IDLE || next == State::RETREAT || next == State::DEFEND) return true;
  if (next == State::RESUPPLY && state_ != State::RETREAT) return true;
  return ticks_in_state_ >= profile_.thresholds.min_ticks_in_state;
}

bool DecisionFsm::attack_push_allowed(const Context & ctx) const
{
  if (!profile_.enable_attack_push || profile_.attack_push.empty()) return false;
  if (!ctx.outpost_alive()) return false;
  if (ctx.overheat_risk()) return false;
  if (ctx.ammo_empty()) return false;
  if (ctx.hp() < profile_.thresholds.hp_low_exit_hysteresis) return false;
  if (ctx.under_attack()) return false;

  // 比赛后期默认使用保守策略（late_game_leading），暂不支持自动检测领先/落后
  if (ctx.late_game() && profile_.late_game_leading.disable_attack_push) return false;

  return true;
}

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
    case State::PATROL:
      switch_stance(StanceCommand::MOBILITY, now_s);
      break;
    case State::DEFEND:
      combat_substate_ = CombatSubState::SCOUT;
      substate_entered_s_ = now_s;
      enemy_lost_at_s_ = 0.0;
      last_hit_at_s_ = 0.0;
      break;
    case State::ATTACK_PUSH:
      switch_stance(StanceCommand::OFFENSIVE, now_s);
      break;
    case State::RESUPPLY:
      switch_stance(StanceCommand::MOBILITY, now_s);
      operation_started_s_ = now_s;
      // 不重置 supply_backup_exhausted_ — 防止补给失败后无限重试振荡
      break;
    case State::RETREAT:
      switch_stance(StanceCommand::DEFENSIVE, now_s);
      operation_started_s_ = now_s;
      break;
  }
}

void DecisionFsm::on_exit(State s)
{
  prev_state_ = s;
  if (s == State::DEFEND) {
    combat_substate_ = CombatSubState::SCOUT;
    attack_source_ = AttackSource::NONE;
    enemy_lost_at_s_ = 0.0;
    last_hit_at_s_ = 0.0;
  }
}

void DecisionFsm::run_behaviour(const Context & ctx, double now_s)
{
  if (goal_sent_ && !goal_arrived_ && ctx.goal_reached()) {
    goal_arrived_ = true;
    waypoint_arrived_s_ = now_s;
  }

  switch (state_) {
    case State::IDLE:
      behave_idle(ctx, now_s);
      break;
    case State::PATROL:
      behave_patrol(ctx, now_s);
      break;
    case State::DEFEND:
      behave_defend(ctx, now_s);
      break;
    case State::ATTACK_PUSH:
      behave_attack_push(ctx, now_s);
      break;
    case State::RESUPPLY:
      behave_resupply(ctx, now_s);
      break;
    case State::RETREAT:
      behave_retreat(ctx, now_s);
      break;
  }
}

void DecisionFsm::behave_idle(const Context &, double) {}

void DecisionFsm::behave_patrol(const Context & ctx, double now_s)
{
  const auto & route = (ctx.late_game() && !profile_.late_game_leading.fallback_patrol.empty())
                         ? profile_.late_game_leading.fallback_patrol
                         : profile_.patrol;

  if (ctx.nav_stuck()) {
    if (!route.empty()) path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    return;
  }
  drive_route(route, now_s);
}

void DecisionFsm::behave_defend(const Context & ctx, double now_s)
{
  if (ctx.under_aerial_attack()) {
    attack_source_ = AttackSource::AERIAL;
    if (
      combat_substate_ != CombatSubState::HARDEN && combat_substate_ != CombatSubState::EVADE_AIR) {
      combat_substate_ = CombatSubState::HARDEN;
      goal_sent_ = false;
    }
  } else if (ctx.under_attack() || ctx.enemy_detected()) {
    attack_source_ = AttackSource::GROUND;
    if (
      combat_substate_ == CombatSubState::HARDEN || combat_substate_ == CombatSubState::EVADE_AIR) {
      combat_substate_ = CombatSubState::SCOUT;
      goal_sent_ = false;
    }
  }
  run_combat_fsm(ctx, now_s);
}

void DecisionFsm::run_combat_fsm(const Context & ctx, double now_s)
{
  const auto prev_sub = combat_substate_;

  if (ctx.under_attack()) last_hit_at_s_ = now_s;

  switch (combat_substate_) {
    case CombatSubState::SCOUT:
      combat_scout(ctx, now_s);
      break;
    case CombatSubState::TRACK:
      combat_track(ctx, now_s);
      break;
    case CombatSubState::ENGAGE:
      combat_engage(ctx, now_s);
      break;
    case CombatSubState::EVADE:
      combat_evade(ctx, now_s);
      break;
    case CombatSubState::HARDEN:
      combat_harden(ctx, now_s);
      break;
    case CombatSubState::EVADE_AIR:
      combat_evade_air(ctx, now_s);
      break;
  }

  if (combat_substate_ != prev_sub) {
    substate_entered_s_ = now_s;
  }
}

void DecisionFsm::combat_scout(const Context & ctx, double now_s)
{
  // SCOUT→TRACK 需至少停留 2 ticks，防止自瞄丢帧导致闪烁切换
  if (ctx.enemy_detected() && now_s - substate_entered_s_ > 0.2) {
    combat_substate_ = CombatSubState::TRACK;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }

  if (ctx.under_attack()) {
    combat_substate_ = CombatSubState::EVADE;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }

  // 没有发现敌人时用 defend_fallback 做防守巡逻
  if (!profile_.defend_fallback.empty()) {
    if (ctx.nav_stuck()) {
      path_idx_ = (path_idx_ + 1) % profile_.defend_fallback.size();
      goal_sent_ = false;
    }
    drive_route(profile_.defend_fallback, now_s);
  }
}

void DecisionFsm::combat_track(const Context & ctx, double now_s)
{
  // TRACK→SCOUT 需至少停留 2 ticks，防止自瞄丢帧导致闪烁切换
  if (!ctx.enemy_detected() && now_s - substate_entered_s_ > 0.2) {
    combat_substate_ = CombatSubState::SCOUT;
    return;
  }
  if (ctx.enemy_distance() < profile_.thresholds.engage_distance) {
    combat_substate_ = CombatSubState::ENGAGE;
    switch_stance(StanceCommand::OFFENSIVE, now_s);
    return;
  }
  if (ctx.under_attack() && now_s - last_hit_at_s_ < 0.5) {
    combat_substate_ = CombatSubState::EVADE;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }

  // 向最近敌人方向移动，缩短距离而非原地等待
  // 需要己方位置已知才计算追击目标（odom 未就绪时跳过）
  if (ctx.nav_stuck()) {
    goal_sent_ = false;
  }
  if (!goal_sent_ && ctx.sentry_pos_valid()) {
    double dx = ctx.nearest_enemy_x() - ctx.sentry_x();
    double dy = ctx.nearest_enemy_y() - ctx.sentry_y();
    double dist = std::hypot(dx, dy);
    if (dist > 0.1) {
      // 追到交火距离 (engage_distance) 即可，不用贴脸
      double target_dist = dist - profile_.thresholds.engage_distance;
      if (target_dist < 0.0) target_dist = 0.0;
      Waypoint pursue;
      pursue.x = ctx.sentry_x() + dx / dist * target_dist;
      pursue.y = ctx.sentry_y() + dy / dist * target_dist;
      publish_single_goal(pursue, now_s);
      last_pursuit_update_s_ = now_s;
    }
  }

  // 追击目标周期性更新：每 1s 重算，跟踪移动敌人
  if (goal_sent_ && ctx.sentry_pos_valid() && now_s - last_pursuit_update_s_ > 1.0) {
    double dx = ctx.nearest_enemy_x() - ctx.sentry_x();
    double dy = ctx.nearest_enemy_y() - ctx.sentry_y();
    double dist = std::hypot(dx, dy);
    if (dist > 0.1) {
      double target_dist = dist - profile_.thresholds.engage_distance;
      if (target_dist < 0.0) target_dist = 0.0;
      Waypoint pursue;
      pursue.x = ctx.sentry_x() + dx / dist * target_dist;
      pursue.y = ctx.sentry_y() + dy / dist * target_dist;
      publish_goal_(pursue);
      goal_sent_ = true;
      waypoint_started_s_ = now_s;
      last_pursuit_update_s_ = now_s;
    }
  }

  // TRACK 超时保护：10s 未进入 ENGAGE → 退回 SCOUT（防止被钓鱼）
  if (now_s - substate_entered_s_ > 10.0) {
    combat_substate_ = CombatSubState::SCOUT;
  }
}

void DecisionFsm::combat_engage(const Context & ctx, double now_s)
{
  if (!ctx.enemy_detected()) {
    combat_substate_ = CombatSubState::SCOUT;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    return;
  }
  if (ctx.enemy_distance() > profile_.thresholds.track_distance) {
    // 敌人跑远了追不上，直接回搜索
    combat_substate_ = CombatSubState::SCOUT;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    return;
  }
  // 敌人退出交火距离但仍在追踪范围内 → 回 TRACK 缩近距离
  // 1.5 倍滞回防止 TRACK↔ENGAGE 振荡，最少停留 2s 才允许此转换
  if (
    ctx.enemy_distance() > profile_.thresholds.engage_distance * 1.5 &&
    now_s - substate_entered_s_ > 2.0) {
    combat_substate_ = CombatSubState::TRACK;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }
  if (ctx.under_attack() && now_s - last_hit_at_s_ < 0.5) {
    combat_substate_ = CombatSubState::EVADE;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    goal_sent_ = false;
    return;
  }
  if (ctx.overheat_risk()) switch_stance(StanceCommand::DEFENSIVE, now_s);

  // ENGAGE 超时保护：30s 后退回 SCOUT
  if (now_s - substate_entered_s_ > 30.0) {
    combat_substate_ = CombatSubState::SCOUT;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
  }
}

void DecisionFsm::combat_evade(const Context & ctx, double now_s)
{
  // 场地没有掩体可躲，最有效的闪避是立刻反击（强制绕过冷却）
  switch_stance(StanceCommand::OFFENSIVE, now_s, /*force=*/true);

  if (!ctx.under_attack() && now_s - last_hit_at_s_ > 1.0) {
    // 1s 没再被打 → 回 SCOUT，自瞄会重新锁定继续打
    combat_substate_ = CombatSubState::SCOUT;
  }
}

void DecisionFsm::combat_harden(const Context & ctx, double now_s)
{
  if (
    ctx.enhanced_defense_remaining() > 0.0 ||
    now_s - substate_entered_s_ <= profile_.thresholds.enhanced_defense_duration_s) {
    switch_stance(StanceCommand::ENHANCED_DEFENSIVE, now_s);
  }
  if (!goal_sent_) publish_single_goal(profile_.safe_cover, now_s);

  if (now_s - substate_entered_s_ > profile_.thresholds.enhanced_defense_duration_s) {
    combat_substate_ = CombatSubState::EVADE_AIR;
    switch_stance(StanceCommand::DEFENSIVE, now_s);
    return;
  }
  if (!ctx.under_aerial_attack()) {
    combat_substate_ = CombatSubState::SCOUT;
    goal_sent_ = false;
  }
}

void DecisionFsm::combat_evade_air(const Context & ctx, double now_s)
{
  switch_stance(StanceCommand::DEFENSIVE, now_s);
  if (ctx.nav_stuck()) goal_sent_ = false;
  if (!goal_sent_) publish_single_goal(profile_.safe_cover, now_s);
  if (!ctx.under_aerial_attack()) combat_substate_ = CombatSubState::SCOUT;
}

void DecisionFsm::behave_attack_push(const Context & ctx, double now_s)
{
  if (ctx.nav_stuck()) {
    if (!profile_.attack_push.empty()) path_idx_ = (path_idx_ + 1) % profile_.attack_push.size();
    goal_sent_ = false;
    return;
  }
  drive_route(profile_.attack_push, now_s);
}

void DecisionFsm::behave_resupply(const Context & ctx, double now_s)
{
  // 到达补给区：重置失败标记，允许下次需要补给时重新尝试
  if (ctx.on_supply_pad()) {
    supply_backup_exhausted_ = false;
    supply_fail_time_ = 0.0;
    return;
  }

  if (!goal_sent_) {
    publish_single_goal(profile_.supply, now_s);
    return;
  }

  if (now_s - operation_started_s_ > profile_.thresholds.resupply_timeout_s) {
    if (
      !profile_.backup_supply_points.empty() && path_idx_ < profile_.backup_supply_points.size()) {
      goal_sent_ = false;
      publish_single_goal(profile_.backup_supply_points[path_idx_++], now_s);
      operation_started_s_ = now_s;
      return;
    }
    supply_backup_exhausted_ = true;
    supply_fail_time_ = now_s;
  }
}

void DecisionFsm::behave_retreat(const Context & ctx, double now_s)
{
  if (!goal_sent_) {
    publish_single_goal(profile_.retreat, now_s);
    return;
  }
  if (ctx.hp() < ctx.max_hp() * 0.05) switch_stance(StanceCommand::ENHANCED_DEFENSIVE, now_s);

  if (ctx.goal_reached()) return;

  const bool nav_timeout =
    ctx.nav_stuck() || (now_s - operation_started_s_ > profile_.thresholds.retreat_timeout_s);
  if (nav_timeout) {
    if (
      !profile_.backup_retreat_points.empty() &&
      path_idx_ < profile_.backup_retreat_points.size()) {
      goal_sent_ = false;
      publish_single_goal(profile_.backup_retreat_points[path_idx_++], now_s);
      operation_started_s_ = now_s;
    } else if (path_idx_ == profile_.backup_retreat_points.size()) {
      // 备用撤退点用完 → 尝试补给区（既能回血又能补弹）
      goal_sent_ = false;
      publish_single_goal(profile_.supply, now_s);
      operation_started_s_ = now_s;
      path_idx_++;
    } else {
      // 补给区也失败 → 最后兜底 safe_cover
      goal_sent_ = false;
      publish_single_goal(profile_.safe_cover, now_s);
      operation_started_s_ = now_s;
    }
  }
}

void DecisionFsm::drive_route(const Route & route, double now_s)
{
  if (route.empty()) return;
  if (path_idx_ >= route.size()) path_idx_ = 0;

  const Waypoint & wp = route[path_idx_];
  if (!goal_sent_) {
    publish_goal_(wp);
    goal_sent_ = true;
    goal_arrived_ = false;
    waypoint_started_s_ = now_s;
    waypoint_arrived_s_ = 0.0;
    return;
  }

  // 路点超时保护：若 stuck_timeout_s 内未到达，跳下一路点
  if (!goal_arrived_ && now_s - waypoint_started_s_ > profile_.thresholds.stuck_timeout_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    return;
  }

  if (!goal_arrived_) {
    return;
  }

  if (now_s - waypoint_arrived_s_ >= wp.dwell_s) {
    path_idx_ = (path_idx_ + 1) % route.size();
    goal_sent_ = false;
    goal_arrived_ = false;
  }
}

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

void DecisionFsm::switch_stance(StanceCommand stance, double now_s, bool force)
{
  if (!send_stance_ || stance == StanceCommand::NONE) return;
  if (stance == last_commanded_stance_) return;

  constexpr double kStanceSwitchCooldownS = 5.0;
  if (
    !force && !can_bypass_stance_cooldown(stance) &&
    now_s - last_stance_command_s_ < kStanceSwitchCooldownS) {
    return;
  }

  send_stance_(stance);
  last_commanded_stance_ = stance;
  last_stance_command_s_ = now_s;
}

bool DecisionFsm::can_bypass_stance_cooldown(StanceCommand stance) const
{
  return stance == StanceCommand::DEFENSIVE || stance == StanceCommand::ENHANCED_DEFENSIVE;
}

}  // namespace sentry_decision
