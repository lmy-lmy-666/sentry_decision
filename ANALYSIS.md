# sentry_decision — 决策系统能力说明

> 最后更新: 2026-07-07 | 目标机器人: 哨兵 (Sentry) | 赛季: RM2026

---

## 1. 概述

`sentry_decision` 是哨兵自主导航决策层，基于纯 C++17 有限状态机（FSM），通过 YAML 配置文件切换战术，无需重新编译。

## 2. 输入源

| 数据源 | 来源 | 用途 |
|--------|------|------|
| 裁判系统 | `rm_serial_driver` → `/referee/*` | 比赛状态、血量、弹药、热量、RFID、金币 |
| 自瞄 | `auto_aim_target_pos`（String 解析） | 近距离敌方检测 → DEFEND/TRACK/ENGAGE |
| 雷达 | `RM2026_BOF_Radar` → `/radar/enemy_positions` | 全局敌方坐标 → 完整敌方感知 + 追击导航 |
| 运动状态 | `motion_manager/motion_state`（结构化优先） + `motion_manager/state`（String fallback） | 导航状态（移动中/卡住/到达/失败） |
| 里程计 | `/odometry` | 己方位置（追击用）+ fallback 模式到达检测 |

## 3. 输出

| 输出 | 目标 | 说明 |
|------|------|------|
| Nav2 goal | `navigate_to_pose` action | 导航路点 |
| `/goal_pose` | PoseStamped topic | Nav2 宕机时的 fallback |
| 姿态指令 | `sentry/command` → `serial_driver` → `0xB6` → STM32 | 进攻/防御/移动/强化姿态 |
| 远程兑换/复活 | `sentry/command`（字段预留） | 待 FSM 驱动 |

---

## 4. 顶层状态机

### 4.1 优先级链（每 tick 重评估，只命中第一个满足的）

```
优先级从高到低：

① IDLE          ← 裁判数据失效 / 比赛未运行
② RETREAT       ← hp < 60（已在 RETREAT 则 hp < 120 才退出，hysteresis 防振荡）
③ DEFEND(空中)  ← 空中威胁
④ RESUPPLY 保持 ← 已在补给中，仍需补给。弹药空时不退出。
                  ├─ 补给耗尽 & hp ≥ 120 → 退出
                  └─ 未耗尽 & (弹药空 or hp < 400) → 保持
⑤ RESUPPLY 进入 ← 需要补给 & 不在冷却期。
                  冷却期：补给耗尽后有弹药时 15s 不重试（弹药空时绕过冷却）
⑥ DEFEND(地面)  ← 敌人检测 / 被攻击
⑦ ATTACK_PUSH   ← 前哨存活 + hp≥180 + 弹药充足 + 不过热 + 不被攻击 + 非后期
⑧ PATROL        ← 默认
```

### 4.2 状态切换防振荡 (can_leave_current_state)

- IDLE 可以随时离开
- 进入 IDLE / RETREAT / DEFEND 总是允许（安全优先）
- RESUPPLY 可以抢断除 RETREAT 外的任何状态
- 其他状态间切换需要 `min_ticks_in_state`（默认 4 ticks = 400ms）

---

## 5. 战斗子状态机

```
                         ┌──────────┐
        from DEFEND ───→│  SCOUT   │←──────────── 超时(10s追踪/30s交火)
                         └────┬─────┘
                              │ enemy_detected
                         ┌────▼─────┐
                    ┌───→│  TRACK   │── distance < 3m ──→ ┌─────────┐
                    │    └────┬─────┘                      │ ENGAGE  │
                    │         │ under_attack               └────┬─────┘
                    │    ┌────▼─────┐  1s 安全                 │ under_attack
                    │    │  EVADE  │─────────→────────┐       │
                    │    └─────────┘                  │  ┌────▼─────┐
                    │                                 └─→│  EVADE  │
                    │                                    └─────────┘
       aerial_threat → ┌──────────┐  15s     ┌──────────────┐
                       │ HARDEN   │─────────→│ EVADE_AIR    │── 无威胁 → SCOUT
                       └──────────┘          └──────────────┘
```

| 子状态 | 行为 | 姿态 | 移动 | 超时 |
|--------|------|------|------|------|
| SCOUT | 沿 defend_fallback 巡逻搜索 | DEFENSIVE | defend_fallback 路线 | — |
| TRACK | 朝敌人方向追击，缩短距离 | DEFENSIVE→OFFENSIVE(接近时) | 向最近敌人方向移动 | 10s → SCOUT |
| ENGAGE | 停住交火 | OFFENSIVE | 不动 | 30s → SCOUT |
| EVADE | 切进攻姿态立刻反击 | OFFENSIVE | 不动 | 1s 安全 → SCOUT |
| HARDEN | 强化防御抵抗空中威胁 | ENHANCED_DEFENSIVE | 向 safe_cover 移动 | 15s → EVADE_AIR |
| EVADE_AIR | 防空闪避 | DEFENSIVE | 向 safe_cover 移动 | — |

---

## 6. 代码架构

```
sentry_decision/
├── CMakeLists.txt                 # ament_cmake, shared_library + rclcpp component
├── package.xml                    # 依赖: rclcpp, nav2_msgs, rm_interfaces, yaml-cpp
├── .clang-format
├── include/sentry_decision/
│   ├── types.hpp                  # 枚举、结构体、阈值默认值
│   ├── context.hpp                # Context: 比赛状态聚合器 (裁判+敌人+导航+位置)
│   ├── profile.hpp                # Profile: YAML → 内存结构
│   ├── fsm.hpp                    # DecisionFsm: 状态转移 + 行为执行
│   └── decision_node.hpp          # DecisionNode: ROS2 节点
├── src/
│   ├── profile.cpp                # YAML 解析 (yaml-cpp)
│   ├── fsm.cpp                    # FSM 实现: select_state + 6 behave_* + 6 combat_*
│   └── decision_node.cpp          # 节点: 订阅、action client、motion_state 解析
├── config/profiles/
│   ├── rmuc_red.yaml / rmuc_blue.yaml / rmul.yaml
├── launch/sentry_decision_launch.py
└── test/fsm_test.cpp              # 31 个 gtest 单元测试
```

### 6.1 types.hpp — 类型定义

| 类型 | 说明 |
|------|------|
| `Waypoint {x, y, dwell_s}` | 导航目标点（不含朝向，云台自瞄独立于底盘） |
| `Route = vector<Waypoint>` | 路径点序列 |
| `State` (enum) | IDLE / PATROL / DEFEND / ATTACK_PUSH / RESUPPLY / RETREAT |
| `CombatSubState` (enum) | SCOUT / TRACK / ENGAGE / EVADE / HARDEN / EVADE_AIR |
| `NavStatus` (enum) | IDLE / MOVING / ARRIVED / STUCK / FAILED |
| `AttackSource` (enum) | NONE / GROUND / AERIAL |
| `StanceCommand` (enum) | NONE / OFFENSIVE / DEFENSIVE / MOBILITY / ENHANCED_* ×3 |
| `EnemyInfo` | 敌人语义快照（detected, nearest_distance, nearest_x/y, count, aerial_threat 等） |
| `SentryInfo` | 哨兵自身语义快照 |
| `Thresholds` | 全部数值阈值，带默认值，YAML 可覆盖 |

### 6.2 context.hpp — Context（状态聚合器）

无 ROS 依赖，纯 C++ 数据聚合。关键方法：

**裁判相关**: `game_running()`, `remain_time()`, `late_game()`, `referee_fresh()`, `hp()`, `hp_critical()`, `hp_low()`, `ammo()`, `ammo_empty()`, `barrel_heat()`, `overheat_risk()`, `needs_resupply()`, `under_attack()`, `attack_direction()`, `gold()`

**感知相关**: `enemy_detected()`, `enemy_distance()`, `nearest_enemy_x()`, `nearest_enemy_y()`, `enemy_count()`, `under_aerial_attack()`, `double_vulnerability_active()`

**导航相关**: `nav_status()`, `nav_stuck()`, `goal_reached()`, `set_nav_status()`

**位置相关**: `sentry_x()`, `sentry_y()`, `set_sentry_position()`（odom 回调填充）

**场地相关**: `outpost_alive()`, `ally_base_hp()`, `on_supply_pad()`, `on_base_gain_point()`

### 6.3 fsm.cpp — 核心 FSM

**tick()**: select_state → can_leave_current_state → on_exit/on_enter → run_behaviour

**select_state()**: 8 级优先级链，每 tick 重新评估。RETREAT/RESUPPLY 有 hysteresis。

**战斗 FSM (run_combat_fsm)**: 6 个子状态，由 behave_defend 驱动。

---

## 7. 稳定性和鲁棒性

### 7.1 防振荡

- **State hysteresis**: RETREAT 进入 hp<60，退出 hp≥120；RESUPPLY 进入 hp<150/弹药空，退出 hp≥400 或弹药补满
- **min_ticks_in_state**: 普通状态最小停留 4 tick（400ms），防止瞬态抖动
- **stance 冷却**: 5s，DEFENSIVE/ENHANCED_DEFENSIVE 可绕过
- **RESUPPLY 冷却**: 补给点全部失败后 15s 不重试（弹药空绕过），防止 PATROL↔RESUPPLY 死循环
- **补给到达重置**: RFID 确认到达补给区后重置失败标记，允许下次正常补给

### 7.2 数据安全

- **裁判 stale 超时**: 3s 无数据 → IDLE，停止所有导航
- **EnemyInfo 过期**: 1s 无新数据自动清除（可配 `enemy_stale_timeout_s`）
- **motion_state 加固**: 词边界检查 + 连续 3 次 idle 才判 FAILED
- **所有数据访问**: `std::optional` 保护，null 时返回安全默认值
- **自瞄+雷达数据融合**: 自瞄做近距离检测（detected/distance），雷达提供全局坐标（nearest_x/y）。自瞄更新时保留雷达坐标不覆盖

### 7.3 导航容错

- **Nav2 action 可用**: 正常走 Action 协议（feedback/result 回调）
- **Nav2 action 不可用**: 自动 fallback 到 PoseStamped topic + odom 距离到达检测
- **导航卡住**: 跳下一个路点
- **cancel_nav 清理**: Nav2 在线和离线模式均正确清理标志位
- **空 route 保护**: `drive_route` 和 `publish_single_goal` 均有空检查和重复发送防护

### 7.4 边界保护

- **Waypoint 到达**: dwell 计时从到达后才开始，未到达不推进
- **path_idx 回绕**: 到达路线末尾自动循环
- **作战超时**: TRACK 10s / ENGAGE 30s → fallback SCOUT
- **撤退多级兜底**: 主撤退点 → 备用撤退点链 → safe_cover → 原地不动
- **补给多级兜底**: 主补给点 → 备用补给点链 → 冷却 → 重试

---

## 8. 修改日志

### 2026-07-07 稳定性修复（本轮）

#### 8.1 RESUPPLY 防死循环振荡
`on_enter(RESUPPLY)` 不再重置 `supply_backup_exhausted_`。新增 15s 冷却期（`supply_retry_cooldown_s`），补给全部失败后有弹药时冷却期内不重试。到达补给区（RFID 确认）后重置。弹药耗尽时绕过冷却。

#### 8.2 EVADE 切进攻姿态反击
`combat_evade()` 不再往掩体跑（RM 场地无掩体），改为切 OFFENSIVE 姿态立刻还击。1s 未被打返回 SCOUT 继续搜索。

#### 8.3 TRACK 主动追击
`combat_track()` 根据雷达提供的敌方全局坐标，计算追击目标点（追到 engage_distance 为止），主动缩短距离。不再原地等待。`EnemyInfo` 新增 `nearest_x`/`nearest_y` 字段。

#### 8.4 EnemyInfo 可配置过期时间
从硬编码 0.5s 改为 YAML 可配 `enemy_stale_timeout_s`，默认 1.0s。

#### 8.5 cancel_nav fallback 清理
Nav2 不可用时 `cancel_nav()` 也正确清理 `nav_goals_active_` 和 `fallback_goal_active_`，防止 on_tick 用旧坐标做到达检测。

#### 8.6 自瞄不覆盖雷达坐标
`auto_aim_target_callback` 更新前从 Context 读取现有 `nearest_x`/`nearest_y`，保留雷达提供的全局坐标。

#### 8.7 导航卡住时 goal 清理
`combat_track` 和 `combat_evade_air` 在 `nav_stuck` 时先设 `goal_sent_=false`，确保新目标能正常发出。

#### 8.8 构建环境修复
- Sentry26 workspace 添加 `.colcon/defaults.yaml`（base-paths: src）
- `rmoss_gz_plugins` 旧版 libgz-math 缓存清理
- `radar_msgs` 符号链接冲突清理
- `.gitignore` 添加 build/install/log，防止子目录构建产物污染 git
- 全仓 30 包编译通过，决策 30/30 测试通过

#### 8.9 数据流验证
- 裁判数据流：外部学校 bag → 决策 PATROL→RESUPPLY→RETREAT→IDLE（验证通过）
- 雷达数据流：雷达测试 bag → EnemyInfo 正常填充（验证通过）

### 2026-07-08 自瞄+雷达融合 + Bug修复

#### 8.10 自瞄+雷达双源融合（读时融合）
- 每个源独立维护检测标记：`aim_detected_` (自瞄) + `radar_has_target_` (雷达)
- 各自独立时间戳 `aim_stamp_` / `radar_stamp_`
- `enemy_detected()` 读时 OR 融合——任一方看到敌人即为真
- 自瞄丢锁不会覆盖雷达的检测（消除竞态）
- `enemy_info_fresh()` 任一源新鲜即有效
- 雷达回调修正 `hypot` 参考系：从到原点改为到哨兵距离
- 雷达回调修正最近敌人选择：恢复 `min_dist` 追踪取真正最近目标

#### 8.11 Bug修复
- **HARDEN→SCOUT goal_sent_ 未重置**：空中威胁消失回搜索时清 `goal_sent_`，防止等 safe_cover 到达才恢复巡逻
- **EVADE OFFENSIVE 被冷却丢弃**：`combat_evade` 中 `switch_stance(OFFENSIVE, force=true)` 强制绕过 5s 冷却
- **串口 stance 下行**：`protocol.yaml` 中 `reserved` → `stance`，`generate.py` 重生成 `packet.hpp`，`sendControlPacket` 填充 `pkt.stance`

#### 8.12 代码审查修复（2026-07-08 本轮）

以下问题通过全面代码审查发现并修复，测试从 30→42：

| 问题 | 修复 | 文件 |
|------|------|------|
| 雷达回调 odom 未就绪时 hypot 从原点错误计算距离 | `sentry_pos_valid()` 检查，未就绪使用首个有效目标 | decision_node.cpp |
| TRACK 追击目标不随敌人移动更新 | 每 1s 周期性重算追击目标，直接 `publish_goal_` 更新 | fsm.cpp |
| 补给耗尽后 RESUPPLY↔PATROL 边界振荡 | `supply_backup_exhausted_` 分支增加 `\|\| ammo_empty()` 保护 | fsm.cpp |
| `publish_single_goal` 未设置 `waypoint_started_s_` | 函数签名增加 `now_s` 参数，统一设置时间戳 | fsm.cpp/hpp |
| `profile.retreat` 路点是死数据 | 撤退链重构：retreat → backup → supply → safe_cover | fsm.cpp |
| ENGAGE 中敌人超出交火距离不移动 | 增加 1.5× 滞回距离 + 2s 停留门槛，回 TRACK 缩近距离 | fsm.cpp |
| `enemy_lost_at_s_` 未在 `on_exit(DEFEND)` 清理 | `on_exit(DEFEND)` 中清零 `enemy_lost_at_s_` 和 `last_hit_at_s_` | fsm.cpp |

#### 8.12 omni_navigation 适配
- RFID topic: `referee/rfid_status` → `referee/rfidStatus` (匹配 serial_driver)
- rm_interfaces 补 `MotionState.msg` + `SentryCommand.msg`
- serial_driver 加 `sentry/command` 订阅
- 删除无效的 String+结构化 motion_state 订阅（omni 无此数据源，Nav2 feedback 兜底）

### 2026-07-05 历史修改

- 基础稳定性修复（now()、RFID、空 route、hysteresis、姿态冷却去重）
- 自瞄 String 输入接入
- 雷达 EnemyPosition 输入接入
- motion_state 结构化 topic（String fallback 保留）
- 0x0120 下行 sentry_cmd（ROS 端已通）
- FSM 战术改进（ATTACK_PUSH 被攻即退、RESUPPLY 失败兜底、RETREAT 备用路线、DEFEND 防守巡逻、late_game 策略、Combat SubState 超时）
- Nav2 fallback 到达检测（odom 距离判断）

---

## 9. 问题状态

| 编号 | 问题 | 状态 |
|------|------|------|
| 9.1 | STM32 未解析 0xB6 | 待电控配合 |
| 9.2 | 0x020D 脱战/姿态反馈未接入 | 待电控 + serial_driver |
| 9.3 | 0x020C 空中威胁来源未接入 | 可选 |
| 9.4 | Profile 坐标未确认 | 待实车地图 |
| 9.5 | 自瞄结构化 Target 开关未开 | 待自瞄同学 |
| 9.6 | 多敌人决策退化 | 低优先级 |
| 9.7 | 己方队友协同 | 远期规划 |
| 9.8 | 热管理优化 | 远期规划 |
| 9.9 | RFID 增益点策略 | 远期规划 |

---

## 10. 可配置参数

### YAML thresholds（默认值，可在 profile YAML 中覆盖）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hp_low` | 150 | 低血量阈值 |
| `hp_critical` | 60 | 致命血量阈值 |
| `hp_low_exit_hysteresis` | 180 | 低血量退出滞后 |
| `hp_critical_exit` | 120 | 致命血量退出滞后 |
| `heat_max` | 240 | 过热阈值 |
| `ammo_min` | 1 | 弹药耗尽阈值 |
| `game_total_time` | 420 | 比赛总时长(s) |
| `min_ticks_in_state` | 4 | 状态最小停留 tick 数 |
| `referee_stale_timeout_s` | 3.0 | 裁判数据过期时间(s) |
| `enemy_stale_timeout_s` | 1.0 | 敌方数据过期时间(s) |
| `supply_retry_cooldown_s` | 15.0 | 补给失败后冷却时间(s) |
| `engage_distance` | 3.0 | 交火距离(m) |
| `track_distance` | 8.0 | 追踪距离(m) |
| `stuck_timeout_s` | 30.0 | 导航卡住超时(s) |
| `resupply_timeout_s` | 60.0 | 单次补给超时(s) |
| `retreat_timeout_s` | 90.0 | 单次撤退超时(s) |
| `enemy_lost_time_s` | 5.0 | 敌人丢失超时(s) |
| `enhanced_defense_duration_s` | 15.0 | 强化防御持续时间(s) |

### ROS2 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `profile_path` | (必填) | YAML tactic 文件路径 |
| `tick_frequency` | 10.0 Hz | FSM tick 频率 |
| `goal_topic` | `/goal_pose` | fallback goal 话题 |
| `nav_action_name` | `navigate_to_pose` | Nav2 action 名称 |
| `goal_frame` | `map` | goal 坐标帧 |
| `goal_reached_distance_tolerance` | 0.25 m | 到达判定距离 |
| `motion_state_topic` | `motion_manager/state` | 运动状态话题（String） |
| `auto_aim_target_topic` | `auto_aim_target_pos` | 自瞄目标话题 |

---

## 11. 编译与运行

```bash
cd ~/Sentry26
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sentry_decision

# 单元测试（需要先 cmake -DBUILD_TESTING=ON 编译）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --packages-select sentry_decision
~/Sentry26/build/sentry_decision/fsm_test

# 红方
ros2 run sentry_decision sentry_decision_node --ros-args \
  -p profile_path:=~/Sentry26/src/sentry_decision/config/profiles/rmuc_red.yaml

# 蓝方
ros2 run sentry_decision sentry_decision_node --ros-args \
  -p profile_path:=~/Sentry26/src/sentry_decision/config/profiles/rmuc_blue.yaml
```

## 12. 其他文档

| 文档 | 内容 |
|------|------|
| `README.md` | 项目概览 |
| `docs/interface_audit.md` | 接口审计 |
| `docs/profile_coordinates.md` | Profile 坐标填写 |
| `docs/测试指南.md` | 测试说明 |
| `docs/changes/` | 历次修改日志 |

---

## 13. 待实战/仿真闭环验证的风险点

以下问题代码静态审查无法发现，需要在仿真或实车环境下闭环测试才能暴露。

### 13.1 数据竞态：自瞄与雷达到达顺序不确定

**问题**: `auto_aim_target_callback` 和 `radar_callback` 各自创建完整 `EnemyInfo` 后调用 `context_.update()` 全量替换。虽然已保护自瞄不覆盖雷达坐标，但两个源到达顺序不确定时，`detected` / `count` / `nearest_distance` 等字段会被最后一个到达的来源覆盖。

**风险**: 雷达报告 3 个敌人，自瞄只报告 1 个近距离目标。如果自瞄在雷达之后到达，`count` 从 3 变成 1，`nearest_distance` 从雷达的全局距离变成自瞄的云台相对距离。F SM 可能丢失多敌人感知。

**验证方式**: 仿真中同时运行雷达和自瞄，观察 `sentry/command` 输出和 FSM 状态日志，确认 EnemyInfo 不会在两个源之间抖动。

**修复方向**: 两个回调不各自创建完整 EnemyInfo，改为更新各自负责的字段子集。Context 提供增量更新接口。

### 13.2 TRACK 追击在真实雷达数据下的流畅性

**问题**: `combat_track` 每次 tick 重新计算追击目标点。如果雷达更新频率低（如 1-5Hz）或坐标有噪声，追击目标可能抖动。频繁更换 Nav2 goal 会增加路径重规划开销。

**风险**: 追击路径不稳定，机器人来回转向，能耗增加；极端情况下可能追丢敌人。

**验证方式**: 仿真中放置移动的敌方机器人，开启雷达数据注入，观察哨兵追击轨迹是否平滑、是否稳定进入 ENGAGE。

**修复方向**: 对追击目标点做低通滤波（滑动平均），或设置最小 goal 更新间隔。

### 13.3 连续战斗中的子状态振荡

**问题**: 战斗子状态 FSM 在边界条件下可能振荡。例如：敌人在 3m 临界距离反复进出 → TRACK↔ENGAGE 来回切。自瞄间歇性丢帧 → enemy_detected 闪烁 → SCOUT↔TRACK 来回切。

**风险**: 姿态命令频繁切换（DEFENSIVE↔OFFENSIVE），导航 goal 反复取消重发，整体效率降低。

**验证方式**: 仿真中模拟边界距离的敌人，观察子状态切换频率。如果某个子状态在 1 秒内切换超过 3 次，需要加 hysteresis。

**修复方向**: 为子状态切换添加 min_ticks 或阈值滞回（如进入 ENGAGE 需要 <2.5m，退出需要 >3.5m）。

### 13.4 motion_state 双通道竞争

**问题**: `motion_state_structured_callback` 和 `motion_state_callback` 同时订阅两个 topic（结构化 + String）。如果 motion_manager 同时发布两个 topic，两者的到达顺序不确定，`nav_status` 可能由最后到达的消息决定。共享的 `idle_detection_counter_` 可能被两个回调交替重置，导致 FAILED 判定延迟或误判。

**风险**: 导航明明在正常运行，但被 String 通道的旧数据误判为 FAILED → 路由跳点。或者导航真的卡住了，但结构化通道一直重置计数器 → 迟迟不跳点。

**验证方式**: 确认 motion_manager 实际只发布一个通道。如果两个都发，在仿真中制造导航卡住场景，验证 FAILED 检测的延迟是否在接受范围内（当前需连续 3 次 idle，约 300ms）。

**修复方向**: 如果 motion_manager 只发一个通道，删除另一个订阅。如果两个都发，将计数器按通道独立。

### 13.5 Nav2 action server 宕机恢复

**问题**: Nav2 宕机时 decision 自动 fallback 到 PoseStamped topic + odom 到达检测。如果 Nav2 恢复，下一帧 `publish_goal` 会切换到 action 模式。但此时上一个 `current_goal_handle_` 可能指向已失效的 goal。

**风险**: Nav2 恢复后第一个 action goal 可能被拒绝（服务端状态不一致）。当前代码中 goal 被拒时设置 NavStatus::FAILED → FSM 走 nav_stuck 重发 goal → 第二次通常成功。影响为一帧延迟。

**验证方式**: 仿真中手动 kill/restart Nav2 action server，确认决策在 1 秒内恢复正常工作。

**修复方向**: 检测到 action server 状态从 down→up 时主动 `async_cancel_all_goals` 清理服务端状态。

### 13.6 补给区 RFID 触发延迟

**问题**: `behave_resupply` 中 `ctx.on_supply_pad()` 依赖 RFID 检测。RFID 是物理感应，可能有 0.5-1 秒延迟。在这段延迟期间，机器人已到达补给区坐标但未被识别为"已到达"。

**风险**: 如果 `resupply_timeout_s` 设置得太短，机器人到达补给区后 RFID 还没触发就被判超时，切到备用补给点。但如果机器人确实没到达（导航误差），适当超时又是必要的。

**验证方式**: 实车测试中在补给区放置 RFID 卡，测量从 goal 到达（Nav2 succeeded）到 RFID 触发（on_supply_pad 变 true）的延迟。调整 `resupply_timeout_s` 使其至少为此延迟的 2 倍。

**修复方向**: 结合 goal 到达（odom 距离）+ RFID 双重确认，而非仅依赖 RFID。

### 13.7 无 motion_state 数据时的行为退化

**问题**: 如果 motion_manager 完全不发数据，`nav_status` 始终为 IDLE。FSM 的路由推进依赖 `goal_arrived_` → `goal_reached()` → `nav_status == ARRIVED`。唯一能设置 ARRIVED 的是 Nav2 feedback 回调和 fallback odom 检测。如果 Nav2 也不可用，odom 检测需要机器人正好走到目标坐标 0.25m 以内。

**风险**: 巡逻路线上机器人一直在移动，但 FSM 认为它从未到达，永远不会推进到下一个路点。机器人会一直朝第一个巡逻点走，撞墙后 motion_manager 可能触发 recovery。

**验证方式**: 仿真中不启动 motion_manager，启动 Nav2，观察是否正常推进路线。

**修复方向**: 添加纯 odom 到达检测（不依赖 fallback goal 标志）作为最后兜底。

### 13.8 串口 0xB6 下行链路未验证

**问题**: decision 发布 `sentry/command` → serial_driver 打包为 0xB6 → STM32 → 裁判系统 0x0120。ROS 端已验证（topic 有发布），但 STM32 解析 0xB6 并转换为 0x0120 发送的链路**完全未测**。

**风险**: 如果 STM32 未实现 0xB6 解析或格式不匹配，所有姿态切换指令（进攻/防御/强化）全部无效。哨兵在所有状态下姿态都不会变化，可能在需要防御时不防御、需要进攻时不开火。

**验证方式**: 最优先的验证项。实车连接 STM32，在 ROS 端发布手动 `sentry/command`，通过裁判系统监控软件确认 0x0120 是否正确接收到 stance_command。

**修复方向**: 如果 STM32 端未实现，需参照 `rm_interfaces/msg/SentryCommand.msg` 格式实现解析逻辑。

---

## 14. omni_navigation 集成记录

> 日期: 2026-07-07 | 分支: fix/main-no-mppi

### 工作空间适配

| 项目 | 改动 |
|------|------|
| RFID topic | `referee/rfid_status` → `referee/rfidStatus`（匹配 omni 的 serial_driver） |
| rm_interfaces | 补 `MotionState.msg` + `SentryCommand.msg` |
| serial_driver | 新增 `sentry/command` 订阅，接收姿态指令 |
| radar_msgs | 作为独立包放入 workspace（决策编译依赖） |

### 导航控制器

omni_navigation 使用 `omni_pid_pursuit_controller`（自研 PID 追踪），不是 MPPI。决策发 goal 到 Nav2 行为树，由该控制器执行。

### 运动状态

omni_navigation 无独立 `motion_manager` 节点。决策通过 Nav2 action feedback 判断 `ARRIVED` / `MOVING` / `FAILED`，不依赖 `motion_manager/state` 话题。`nav_stuck` 检测不可用（无卡住感知），但 Nav2 自带超时恢复。

### 数据流验证

- 裁判数据：外部学校 bag → PATROL→RESUPPLY→RETREAT→IDLE ✅
- 单元测试：42/42 ✅
