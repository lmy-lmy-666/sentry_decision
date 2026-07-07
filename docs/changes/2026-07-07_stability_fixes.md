# 2026-07-07 稳定性修复与全面审查

## 修复内容

### 1. RESUPPLY 防死循环振荡（fsm.cpp / types.hpp）
- `on_enter(RESUPPLY)` 不再重置 `supply_backup_exhausted_`
- 新增 `supply_retry_cooldown_s` 阈值（默认 15s），补给全部失败后冷却期内有弹药时不重试
- 弹药耗尽时绕过冷却
- 到达补给区（RFID on_supply_pad）时重置失败标记
- `select_state()` 加冷却判断逻辑

### 2. EVADE 切进攻姿态反击（fsm.cpp）
- `combat_evade()` 改为切 OFFENSIVE 姿态立刻还击
- 不再往 safe_cover 移动（RM 场地无掩体可躲）

### 3. TRACK 主动追击（fsm.cpp / context.hpp / types.hpp / decision_node.cpp）
- `EnemyInfo` 新增 `nearest_x` / `nearest_y` 字段
- `Context` 新增 `nearest_enemy_x()` / `nearest_enemy_y()` 和 `sentry_x()` / `sentry_y()`
- `radar_callback` 填充最近敌人坐标
- `odometry_callback` 填充己方位置到 Context
- `combat_track()` 根据雷达坐标计算追击目标点，追到 engage_distance 为止
- `combat_track()` 超时从 20s 改为 10s
- `combat_engage()` 敌人跑远 (>8m) 改为直接回 SCOUT

### 4. EnemyInfo 过期可配置（context.hpp / types.hpp / profile.cpp）
- 硬编码 0.5s 改为可配 `enemy_stale_timeout_s`，默认 1.0s
- 3 个 YAML profile 均更新

### 5. cancel_nav fallback 清理（decision_node.cpp）
- Nav2 不可用时 `cancel_nav()` 也清理 `nav_goals_active_` 和 `fallback_goal_active_`

### 6. 自瞄不覆盖雷达坐标（decision_node.cpp）
- `auto_aim_target_callback` 写入前从 Context 读取现有 nearest_x/y，保留雷达全局坐标

### 7. nav_stuck 时 goal 清理（fsm.cpp）
- `combat_track` 和 `combat_evade_air` 在 nav_stuck 时先设 `goal_sent_=false`

### 8. 数据流验证
- 裁判数据流：使用外部学校 bag 验证 PATROL→RESUPPLY→RETREAT→IDLE 链路
- 雷达数据流：使用雷达测试 bag 验证 EnemyInfo 正常填充

## 审查结论

- 状态机优先级链正确，状态间无冲突
- 所有数据访问有 optional 保护
- 边界条件均有处理（hp 临界、弹药空、路线空、nav 宕机等）
- 30/30 单元测试通过

## 待实战闭环验证

详见 ANALYSIS.md 第 13 节（8 个风险点）。
