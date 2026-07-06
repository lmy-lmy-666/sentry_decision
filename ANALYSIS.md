# sentry_decision — 决策系统当前能力说明

> 最后更新: 2026-07-06 | 目标机器人: 哨兵 (Sentry) | 赛季: RM2026

---

## 1. 概述

`sentry_decision` 是哨兵自主导航决策层，基于纯 C++17 有限状态机（FSM），通过 YAML 配置文件切换战术，无需重新编译。

## 2. 当前输入

| 数据源 | 来源 | 用途 |
|--------|------|------|
| 裁判系统 | `rm_serial_driver` → `/referee/*` | 比赛状态、血量、弹药、热量、RFID、金币 |
| 自瞄 | `auto_aim_target_pos`（String 解析） | 近距离敌方检测 → DEFEND/TRACK/ENGAGE |
| 雷达 | `RM2026_BOF_Radar` → `/radar/enemy_positions` | 全局敌方坐标 → 完整敌方感知 |
| 运动状态 | `motion_manager/motion_state`（结构化） + `motion_manager/state`（String fallback） | 导航状态（移动中/卡住/到达/失败） |
| 里程计 | `/odometry` | fallback 模式到达检测 |

## 3. 当前输出

| 输出 | 目标 | 说明 |
|------|------|------|
| Nav2 goal | `navigate_to_pose` action | 导航路点 |
| `/goal_pose` | PoseStamped topic | fallback 模式 |
| 姿态指令 | `sentry/command` → `serial_driver` → `0xB6` → STM32 | 进攻/防御/移动/强化姿态 |
| 远程兑换 | `sentry/command`（字段预留） | 弹药/血量兑换请求 |

## 4. 顶层状态

| 状态 | 触发条件 | 行为 |
|------|---------|------|
| IDLE | 比赛未运行/裁判数据失效 | 停止导航 |
| PATROL | 默认，无更高优先级任务 | patrol 路线巡逻（后期切换 fallback_patrol） |
| DEFEND | 敌方检测/被攻击/空中威胁 | SCOUT→TRACK→ENGAGE→EVADE，空中→HARDEN→EVADE_AIR |
| ATTACK_PUSH | 前哨站存活+血量弹药充足+不过热+不被攻击+后期不前压 | attack_push 路线前压 |
| RESUPPLY | 弹药空/血量低，且无敌人在附近 | 去补给区，失败后回 PATROL |
| RETREAT | 血量危急(hp<60) | 去安全点，卡住换备用点，血量恢复后退出 |

## 5. 子状态

| 子状态 | 说明 | 超时 |
|--------|------|------|
| SCOUT | 搜索：防守巡逻（defend_fallback），发现敌人→TRACK | — |
| TRACK | 追踪：距离>3m跟踪，<3m→ENGAGE，被击中→EVADE | 20s |
| ENGAGE | 交战：停住射击，敌人消失→SCOUT，被击中→EVADE | 30s |
| EVADE | 闪避：向 safe_cover 移动，2s安全→SCOUT | — |
| HARDEN | 强化防御：抵抗空中威胁，15s后→EVADE_AIR | 15s |
| EVADE_AIR | 防空闪避：不规则移动，空中威胁消失→SCOUT | — |

## 6. 稳定性保护

- **state hysteresis**：RETREAT 进入 hp<60，退出 hp>120；RESUPPLY 进入 hp<150，退出 hp>180
- **min_ticks_in_state**：普通状态最小停留 4 tick（防振荡）
- **stance 冷却**：5s，DEFENSIVE/ENHANCED_DEFENSIVE 可绕过
- **EnemyInfo 过期**：0.5s 无新数据自动清除
- **motion_state 加固**：词边界检查 + 连续 3 次 idle 才判失败
- **敌人在时不进 RESUPPLY**：避免战斗和补给来回切
- **Nav2 fallback**：action down 时用 odom 自行判断到达

## 7. 测试状态

```text
GTest: 19/19 通过
Lint: 7/7 通过
编译: rm_interfaces + radar_msgs + sentry_decision + rm_serial_driver + sentry_motion_manager 通过
实机: 雷达 ROS 桥接数据流经验证
```

---

## 2. 代码架构

```
sentry_decision/
├── CMakeLists.txt               # ament_cmake, 编译为 shared_library + rclcpp component
├── package.xml                  # 依赖: rclcpp, nav2_msgs, rm_interfaces, yaml-cpp
├── .clang-format
├── include/sentry_decision/
│   ├── types.hpp                # 枚举、结构体、阈值默认值
│   ├── context.hpp              # Context: 比赛状态聚合器 (裁判+敌人+导航)
│   ├── profile.hpp              # Profile: YAML → 内存结构, load_profile()
│   ├── fsm.hpp                  # DecisionFsm: 状态转移 + 行为执行
│   └── decision_node.hpp        # DecisionNode: ROS2 节点, 订阅/发布/定时器
├── src/
│   ├── profile.cpp              # YAML 解析 (yaml-cpp)
│   ├── fsm.cpp                  # FSM 实现: select_state + 6 behave_* + 6 combat_*
│   └── decision_node.cpp        # 节点实现: 订阅、action client、motion_state 解析
├── config/profiles/
│   ├── rmuc_red.yaml
│   ├── rmuc_blue.yaml
│   └── rmul.yaml
├── launch/sentry_decision_launch.py
└── test/fsm_test.cpp            # 20 个 gtest 单元测试
```

### 2.1 types.hpp — 类型定义

| 类型 | 说明 |
|------|------|
| `Waypoint {x, y, dwell_s}` | 导航目标点，不含朝向（云台自瞄独立于底盘） |
| `Route = vector<Waypoint>` | 路径点序列 |
| `State` (enum) | IDLE / PATROL / DEFEND / ATTACK_PUSH / RESUPPLY / RETREAT |
| `CombatSubState` (enum) | SCOUT / TRACK / ENGAGE / EVADE / HARDEN / EVADE_AIR |
| `NavStatus` (enum) | IDLE / MOVING / ARRIVED / STUCK / FAILED |
| `AttackSource` (enum) | NONE / GROUND / AERIAL |
| `StanceCommand` (enum) | NONE / OFFENSIVE / DEFENSIVE / MOBILITY / ENHANCED_* ×3 |
| `EnemyInfo` | 敌人语义快照（detected, distance, count, near_base 等） |
| `SentryInfo` | 哨兵自身语义快照（disengaged, stance, enhanced_timer） |
| `Thresholds` | 全部数值阈值，带默认值 |

### 2.2 context.hpp — Context（状态聚合器）

无 ROS 依赖，纯 C++ 数据聚合。通过 `update()` 重载方法注入裁判/敌人/哨兵信息：

- **裁判信息**: `GameStatus`, `RobotStatus`, `RfidStatus`, `GameRobotHP`
- **感知信息**: `EnemyInfo`, `SentryInfo`
- **导航状态**: `NavStatus` 由 `DecisionNode` 的回调设置
- **时间**: `set_now(double)` 注入仿真/墙上时间（支持 sim time）

关键推导方法:
- `game_running()`: 比赛进行中 + 剩余时间合法
- `needs_resupply()`: 弹药空 || HP 低于阈值
- `hp_critical()`: HP 低于致命线
- `under_attack()`: 扣血原因为 ARMOR_HIT
- `attack_direction()`: 根据被打装甲板 ID 推断方向
- `late_game()`: 剩余时间 < 60s

### 2.3 profile.hpp / profile.cpp — YAML 配置

`load_profile(path)` 解析 YAML 生成 `Profile` 结构体：

```yaml
thresholds: {hp_low, hp_critical, heat_max, ammo_min, ...}
patrol: [{x, y, dwell_s}, ...]
attack_push: [{x, y, dwell_s}, ...]
defend_fallback: [...]    # 解析了但 FSM 里没使用
supply: {x, y}
retreat: {x, y}
safe_cover: {x, y}
enable_attack_push: bool
backup_supply_points: [...]
backup_retreat_points: [...]  # 解析了但 FSM 里没使用
late_game_leading:  {disable_attack_push, fallback_patrol}  # 解析了但 FSM 没使用
late_game_trailing: {disable_attack_push, fallback_patrol}  # 解析了但 FSM 没使用
```

### 2.4 fsm.hpp / fsm.cpp — 核心 FSM

**输入**: `Context & ctx`, `double now_s` (每 tick 调用)  
**输出**: 调用 `GoalPublisher` 发 waypoint / `StanceSender` 发 stance / `NavCanceller` 取消导航

**状态转移优先级** (select_state 顺序):

```
referee 掉线/比赛未开始  → IDLE
当前在 RETREAT 且 HP 仍很低  → RETREAT (滞回)
HP 致命                   → RETREAT
空中威胁                  → DEFEND
当前在 RESUPPLY 且仍需补给  → RESUPPLY (滞回)
需要补给                  → RESUPPLY
敌人发现/正在被攻击        → DEFEND
允许推进                  → ATTACK_PUSH
默认                      → PATROL
```

**状态切换 hysteresis (can_leave_current_state)**:
- 可以无条件离开 IDLE、进入安全状态 (IDLE/RETREAT/DEFEND)
- RESUPPLY 可以抢断除 RETREAT 外的任何状态
- 其他状态间切换需要 `ticks_in_state_ >= min_ticks_in_state`（防止抖动）

**route 推进逻辑 (drive_route)**:
```
goal 未发出 → 发送当前 waypoint → 等待到达 → 等待 dwell_s → 推进 index → 循环
```

**战斗子状态 FSM (run_combat_fsm)**:

```
                          ┌──────────┐
         from DEFEND ───→│  SCOUT   │
                          └────┬─────┘
                               │ enemy_detected
                          ┌────▼─────┐
                          │  TRACK   │──────── distance < engage ──→ ┌─────────┐
                          └────┬─────┘                                │ ENGAGE  │
                               │ under_attack                         └────┬─────┘
                          ┌────▼─────┐  2s safe                          │ under_attack
                          │  EVADE   │─────────→ SCOUT                  │
                          └──────────┘                              ┌────▼─────┐
                                                                    │  EVADE   │
        aerial_threat ──→ ┌──────────┐                              └──────────┘
                          │  HARDEN  │── timer_expired ──→ ┌──────────────┐
                          └──────────┘                     │ EVADE_AIR    │── no threat → SCOUT
                                                           └──────────────┘
```

### 2.5 decision_node.hpp / decision_node.cpp — ROS2 节点

**订阅:**
| Topic | 类型 | 用途 |
|-------|------|------|
| `referee/game_status` | GameStatus | 比赛阶段/剩余时间 |
| `referee/robot_status` | RobotStatus | HP/弹药/热量/扣血原因 |
| `referee/rfid_status` | RfidStatus | RFID 增益点/补给区 |
| `referee/all_robot_hp` | GameRobotHP | 前哨站/基地血量 |
| `motion_manager/state` | String | 解析运动状态 (stuck/recovery/mode) |

**发布 / Action:**
- **Nav2 Action**: `navigate_to_pose` (带 `PoseStamped` topic fallback)
- **Topic**: `/goal_pose` (fallback 模式)
- **Stance**: STUB，仅 log 输出

**关键参数 (ros2 param):**
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `profile_path` | (必填) | YAML tactic 文件路径 |
| `tick_frequency` | 10.0 Hz | FSM tick 频率 |
| `goal_topic` | `/goal_pose` | fallback goal 话题 |
| `nav_action_name` | `navigate_to_pose` | Nav2 action 名称 |
| `goal_frame` | `map` | goal 坐标帧 |
| `goal_reached_distance_tolerance` | 0.25 m | 到达判定距离 |

---

## 3. 当前存在的问题（需修复）

### 3.1 :green_circle: `send_stance_command` 已打通（ROS 端）

```cpp
// decision_node.cpp 发布 sentry/command (SentryCommand)
// serial_driver 订阅后打包为 0xB6 自定义串口包下发 STM32
```

**已完成**:
- 新增 `rm_interfaces/msg/referee/SentryCommand.msg`，覆盖 0x0120 全部字段（confirm_revive、confirm_instant_revive、projectile_exchange_amount、remote_projectile_exchange_count、remote_hp_exchange_count、stance_command、activate_energy_mechanism）。
- `sentry_decision` 的 `send_stance_command()` 现已发布 `sentry/command` topic，不再是纯日志 STUB。
- `serial_driver` 新增 `HEADER_SENTRY_CMD = 0xB6` 下行包，订阅 `sentry/command` 并打包通过串口发送给 STM32。
- 已通过 ROS2 topic 模拟验证整条链路。

**剩余依赖**:
- STM32 侧需要新增 `0xB6` 包解析，并转换为 RM2026 裁判系统 `0x0120 sentry_cmd` 发送。这部分属于电控固件，不在 ROS 端修改范围内。
- `sentry/command` 目前只填充 `stance_command` 字段，远程兑换血量/弹量、确认复活等字段尚未由 FSM 逻辑驱动。

### 3.2 :yellow_circle: `EnemyInfo` 已接入自瞄 + 雷达双源

```cpp
// 近距离：订阅 auto_aim_target_pos (std_msgs/String: "x,y,valid,id")
// 全局：  订阅 radar/enemy_positions (rm_interfaces/EnemyPosition: x, y, robot_type)
```

**已完成**:
- 自瞄输入（近距离）：`DecisionNode` 新增 `auto_aim_target_topic` 参数，默认 `auto_aim_target_pos`。解析 `x,y,valid,id` 转为 `EnemyInfo`。
- 雷达输入（全局坐标）：`DecisionNode` 新增 `radar/enemy_positions` 订阅，聚合 6 个 `EnemyPosition` 为 `EnemyInfo`。
- 雷达站（`RM2026_BOF_Radar`）新增 `ros_publisher.py`，惰性初始化 ROS2 发布桥接。
- `rm_interfaces` 新增 `EnemyPosition.msg`。

**剩余影响/限制**:
- 自瞄仍是字符串解析，坐标是云台坐标系，只能做近距离（~8m）判断。
- 雷达给全局 map 坐标，可判断 `enemy_near_base/near_outpost`（需 profile 配置 base/outpost 参考坐标）。
- `under_aerial_attack()` 和 `double_vulnerability_active()` 仍缺真实雷达/裁判系统来源（需 0x020C）。

**优先级**: 中。短期已可支撑近距离 `DEFEND/TRACK/ENGAGE`；中期建议让自瞄新增结构化 `rm_interfaces/msg/vision/Target` topic，长期仍需要雷达/0x0301 或全局敌方坐标。

### 3.3 :red_circle: `SentryInfo` 从未被填充

```cpp
// 没有 subscriber 或 serial parsing 填充 SentryInfo
```

**影响**:
- `stance_expiring()` 固定返回 false
- `enhanced_defense_remaining()` 返回 0.0（导致 HARDEN 计时完全依赖 FSM 内部 clock，无法反映实际增强防御剩余时间）

**优先级**: 中。需要串口 0x0120 uplink 或对应 ROS topic。

### 3.4 :yellow_circle: motion_state 解析已加固，但仍建议结构化消息

```cpp
// decision_node.cpp: motion_state_callback 和 contains_token
```

**已完成**:
- `contains_token()` 增加词边界检查（左右需空格/逗号/字符串首尾）。
- `mode=idle` 需连续 3 次才判 `FAILED`，防止瞬态误报。

**剩余**: 仍建议 `motion_manager` 发布结构化消息。

### 3.5 :yellow_circle: late_game 策略已启用

```cpp
// fsm.cpp: attack_push_allowed() 使用 profile_.late_game_leading.disable_attack_push
//          behave_patrol() 在后期使用 fallback_patrol 路线
```

**已完成**:
- `attack_push_allowed()` 在后期读取 `late_game_leading.disable_attack_push` 配置。
- `behave_patrol()` 在后期自动切换 `fallback_patrol` 路线（若配置）。

**剩余**: 暂不支持自动检测领先/落后（需敌方基地血量，当前 rm_interfaces 无此字段）。

### 3.6 :green_circle: defend_fallback 已启用

`combat_scout()` 在未发现敌人时，使用 `defend_fallback` 路线进行防守巡逻。

### 3.7 :green_circle: backup_retreat_points 已启用

`behave_retreat()` 在 nav_stuck 或 timeout 时，会依次尝试 `backup_retreat_points` 和 `safe_cover`。

### 3.8 :green_circle: topic fallback 路径到达检测

```cpp
// on_tick() 中 fallback 模式下用 odom 距离判断到达：
// hypot(current_x - goal_x, current_y - goal_y) <= 0.25 → ARRIVED
```
已添加 odom 订阅和 fallback 距离检测，不再依赖 Nav2 feedback。


---

## 3.9 问题汇总清单

| 编号 | 问题 | 状态 | 优先级 |
|------|------|------|--------|
| 3.1 | `send_stance_command` 0x0120 下行 | ROS 端已打通，STM32 待配合 | 高 |
| 3.2 | `EnemyInfo` 自瞄输入 | 已接入 String，缺结构化消息和全局坐标 | 中 |
| 3.3 | `SentryInfo` / 0x020D | 未接入，无姿态/脱战/强化剩余时间反馈 | 高 |
| 3.4 | `motion_state` 字符串解析 | 已加固（边界检查+连续确认），仍建议结构化 | 中 |
| 3.5 | `late_game` 策略 | 已启用，暂缺敌方基地血量无法自动判断领先/落后 | 中 |
| 3.6 | `defend_fallback` route | 已使用 | **已解决** |
| 3.7 | `backup_retreat_points` | 已使用 | **已解决** |
| 3.8 | topic fallback 到达检测 | 已添加 odom 距离检测 | **已解决** |
| 新 | `ATTACK_PUSH` 被攻击时退出 | 已增加 `!under_attack()` 条件 | **已解决** |
| 新 | `RESUPPLY` 失败兜底 | 补给全部超时后仅 hp_critical 保持 | **已解决** |
| 新 | `RETREAT` stuck/timeout 兜底 | 依次尝试 backup → safe_cover | **已解决** |
| 新 | `Combat SubState` 超时保护 | TRACK 20s / ENGAGE 30s → fallback SCOUT | **已解决** |
| 新 | `裁判 stale timeout` 参数化 | 从硬编码 3.0s 改为 `referee_stale_timeout_s` 参数 | **已解决** |
| 5.1 | Nav2 action 宕机恢复 | fallback 模式已加 odom 到达检测与 goal 清理 | **已解决** |
| 5.4 | 多敌人决策退化 | 只知道最近敌人，无法评估整体威胁 | 低 |
| 5.5 | dwell 时间窗口 | 巡逻驻留期间可能延迟响应 | 低 |
| 5.6 | Waypoint 无朝向 | Nav2 可能选择次优路径方向 | 低 |


---

## 4. 可优化方案

### 4.1 :bulb: 敌人状态估计（Kalman/Particle Filter）

当前 `EnemyInfo` 只有 `nearest_distance`。可扩展为含速度估计的结构体，实现：
- 预判敌人移动路径 → ENGAGE 状态下主动拦截而非被动跟随
- 在 EVADE 状态下选择远离敌人方向而非固定 retreat 点

### 4.2 :bulb: attack_direction 驱动的动态撤退方向

```cpp
// context.hpp:119-133 attack_direction() 已实现
double attack_direction() const {
  switch (hit_armor_id()) {
    case 0: return 0.0;             // 前方中弹 → 正后方撤退
    case 1: return π/2;              // 左侧中弹 → 右后方撤退
    case 2: return π;                // 后方中弹 → 前方撤退
    case 3: return -π/2;             // ...
  }
}
```

RETREAT 时可计算动态撤退目标点 = `current_pose + retreat_distance * direction_vector`，而非固定 `profile_.retreat` 点。

### 4.3 :bulb: 经济感知决策

`Context::gold()` 已实现但没有被任何决策逻辑使用。可以：
- 金币足够时购买更快射击/恢复 → 自动切换为 ENHANCED_OFFENSIVE
- 金币不足时更保守 → 禁用 attack_push

### 4.4 :bulb: 己方队友协同

当前 FSM 只知道己方机器人血量和己方前哨站/基地血量，不知道己方英雄、工程、步兵的实时位置，因此还不能判断队友是否在守基地、前压或占补给区。

可优化方向：
- 己方英雄/步兵前压时，哨兵可提高 ATTACK_PUSH 意愿。
- 己方基地附近无人且敌方接近时，哨兵优先 DEFEND/RETREAT 到基地防守点。
- 工程机器人在补给区或兑换区附近时，哨兵避免抢占补给路线。
- 己方主要输出机器人残血时，哨兵减少前压，补防关键通道。

依赖接口：
- 0x020B 己方地面机器人位置。
- GameRobotHP 己方机器人血量。
- 雷达/自瞄/交互数据提供的敌方位置。

### 4.5 :bulb: 热管理优化

当前只有 `overheat_risk()` 二元判断。可优化为：
- 热量 < 50% → 允许连续射击
- 热量 50-80% → 限制射击频率 (切换到 burst mode)
- 热量 > 80% → 触发 EVADE 主动冷却

### 4.6 :bulb: Combat SubState 超时保护

当前 SCOUT / TRACK / ENGAGE 子状态没有退出超时——如果敌人距离卡在阈值边界，可能无限抖振。建议添加：
- ENGAGE 最长时间 (如 30s) → auto fallback to SCOUT
- TRACK 敌人但长时间不进入 ENGAGE → auto fallback to PATROL

### 4.7 :bulb: RFID 增益点策略

当前 `on_base_gain_point()` 已实现但未被使用。可添加：
- 巡逻路线包含增益点 → 每 N 圈刷新一次 buff
- ATTACK_PUSH 前检查是否有活跃增益

---

## 5. 潜在风险与边界情况

### 5.1 :warning: FSM 无异常恢复机制

如果 Nav2 action server 宕机，`publish_goal()` fallback 到 topic 模式，但：
- fallback 模式没有到达回调 → 永远不会 `goal_arrived_=true`
- resume 后 ACTION 重新可用时，`current_goal_handle_` 可能指向上一个已失效的 goal

### 5.2 :warning: RESUPPLY 仅依赖 waypoint 到达

`behave_resupply()` 发布 supply 点 goal，等待 `goal_arrived_`。但：
- 实际补给通过 RFID 确认 (`on_supply_pad()`)
- 如果导航到达了但 RFID 没触发（sensor 误差），会一直卡在 RESUPPLY
- `resupply_timeout_s` 只能切换到 backup 点，没有 fallback 到 PATROL 的路径

### 5.3 :warning: ATTACK_PUSH 无撤退路径

如果推进过程中被反击（hp 快速下降），只有等到 hp_critical 才会触发 RETREAT。hp_low 期间（150-60）仍然继续推进，这在敌方基地附近是非常危险的。

### 5.4 :warning: 多个 enemy 时的决策退化

```cpp
// fsm.cpp:237 combat_scout — enemy_detected 只检查 bool
if (ctx.enemy_detected()) { combat_substate_ = CombatSubState::TRACK; }
```

多个敌人时 `enemy_distance()` 返回最近的，但 FSM 不知道是否有更远的敌人正在接近（无法评估整体威胁）。

### 5.5 :warning: dwell_s 导致的时间窗口

巡逻到达 waypoint 后停留 `dwell_s` 秒，期间如果比赛事件发生（enemy detected / hp drop），`can_leave_current_state` 检查 `min_ticks_in_state` 可能导致响应延迟（10Hz tick × 4 ticks = 400ms 最坏延迟）。

### 5.6 :warning: Waypoint 无朝向信息

```cpp
// types.hpp:27-32
struct Waypoint { double x, y, dwell_s; };  // 无朝向
// decision_node.cpp:113
goal_msg.pose.pose.orientation.w = 1.0;     // 固定四元数
```

Nav2 可能选择次优路径方向（如倒着到达），因为目标无朝向约束。

### 5.7 :warning: 裁判数据时效性

```cpp
// context.hpp:55 — 3秒 stale 阈值硬编码
bool game_status_fresh() const { return ... (now() - game_status_stamp_) < 3.0; }
```

如果裁判系统 10Hz 降到 5Hz，正常刷新也会视为 stale → 强制 IDLE。

### 5.8 :warning: 仿真 vs 实车差异

- `Context::set_now()` 在仿真中注入 sim time，实车使用 wall time
- `motion_state` 解析的 token 可能因 `motion_manager` 版本不同而改变
- Stance 指令当前为 STUB，在实车上需要对接串口协议 0x0120

---

## 6. 单元测试覆盖情况

`test/fsm_test.cpp` — 20 个 gtest 用例:

| 测试 | 验证内容 |
|------|---------|
| 基础状态转移 | IDLE, PATROL, ATTACK_PUSH, RESUPPLY, RETREAT 的条件触发 |
| 优先级 | 致命 HP > 补给需求; 敌人发现 > 推进 |
| Route 推进 | 到达+dwell 后 index 前进; 循环; 到达前不前进 |
| 战斗子状态 | 距离驱动 TRACK→ENGAGE; 空中威胁→HARDEN |
| 比赛结束 | GAME_OVER → IDLE |
| Stale 检测 | 裁判超时 → IDLE |
| Late game | 尾段禁用 attack_push |
| HP 门槛 | HP 不够不触发 attack_push |
| Stance | ENHANCED_DEFENSIVE 绕过冷却; 普通 stance 受冷却控制 |
| 备用补给点 | timeout 后切换 |

**缺失的测试覆盖:**
- EVADE 子状态
- EVADE_AIR 子状态
- Stuck → route_idx 跳转
- Retreat timeout 后行为
- Nav 取消逻辑
- Goal 被 Nav2 拒绝后的恢复

---

## 7. 接入方向

### 7.1 最短路径（让 FSM 实际运转）

1. **EnemyInfo 自瞄输入已接入**: 当前已兼容 `auto_aim_target_pos` (`std_msgs/String`, `x,y,valid,id`)，可支撑近距离 `DEFEND/TRACK/ENGAGE`；后续仍建议改结构化消息并接雷达/全局敌方坐标
2. **对接 stance 下发**: 实现串口 0x0120 downlink 或 `rmua19_robot_base` 的 stance command 接口
3. **对接 motion_state**: 改用结构化消息

### 7.2 结构化 motion_state 建议

```cpp
// 建议在 rm_interfaces 新增
struct MotionState {
  uint8 mode;            // IDLE=0, NAVIGATION=1, RECOVERY=2
  uint8 recovery_phase;  // STRAIGHT_RELEASE=0, ARC_ESCAPE=1, ...
  bool emergency_stop;
  bool has_fresh_command;
};
```

### 7.3 决策级联调流程

```
sentry_decision (this pkg)
  ├── goal → Nav2 → motion_manager → odom
  ├── stance → serial_driver → robot_base
  ├── EnemyInfo ← radar/AutoAim (待接入)
  └── SentryInfo ← serial_driver 0x0120 uplink (待接入)
```

---

## 8. 文件清单

| 文件 | 行数 | 说明 |
|------|------|------|
| `types.hpp` | 166 | 所有枚举、结构体、阈值定义 |
| `context.hpp` | 183 | 状态聚合器(头文件实现) |
| `profile.hpp` | 55 | YAML 解析接口 + Profile 结构体 |
| `fsm.hpp` | 103 | DecisionFsm 接口 |
| `decision_node.hpp` | 83 | ROS2 Node 接口 |
| `profile.cpp` | 121 | YAML → Profile 解析实现 |
| `fsm.cpp` | 425 | 完整 FSM 逻辑 |
| `decision_node.cpp` | 270 | ROS 订阅/发布/Nav2 Action Client |
| `fsm_test.cpp` | 356 | 20 个 gtest 单元测试 |
| `rmuc_red.yaml` | 51 | 红方 RMUC 配置 |
| `rmuc_blue.yaml` | ~50 | 蓝方 RMUC 配置 |
| `rmul.yaml` | ~50 | RMUL 配置 |
| `sentry_decision_launch.py` | 91 | Launch 文件 |

---

## 8. 修改日志

### 2026-07-05 Bug 修复

#### 8.1 combat_harden 计时修正
`combat_harden()` 的强化防御持续时间改用 `substate_entered_s_` 而非 `state_entered_s_`。修复了 HARDEN 从 DEFEND 进入时间开始算而非从真正进入 HARDEN 子状态开始算的问题。

#### 8.2 EnemyInfo 数据过期保护
`Context` 新增 `enemy_info_fresh()`，超过 0.5s 未更新 EnemyInfo 视为过期。所有敌人查询先过新鲜度检查，防止自瞄断连后 FSM 误判敌人永远存在。

#### 8.3 RESUPPLY 敌人让路
`select_state()` 的 RESUPPLY 保持条件新增 `!ctx.enemy_detected()`。敌人出现时不再强制保持 RESUPPLY，让 guard chain 可正常回退到 DEFEND。

详见: `docs/changes/2026-07-05_bug_fixes.md`

### 2026-07-05 新接口接入

#### 8.4 EnemyInfo 自瞄 + 雷达双源
- 近距离：`auto_aim_target_pos`（自瞄云台坐标，字符串解析）→ EnemyInfo。
- 全局：`radar/enemy_positions`（雷达 0x0A01 广播波解析，map 坐标）→ EnemyInfo。
- 新增 `rm_interfaces/msg/EnemyPosition.msg`。
- 雷达站 `RM2026_BOF_Radar` 新增惰性 ROS2 桥接（`ros_publisher.py`）。

#### 8.5 motion_state 结构化
- 新增 `rm_interfaces/msg/MotionState.msg`（含 enum 常量）。
- `sentry_motion_manager` 新增 `motion_manager/motion_state` 结构化 topic。
- `sentry_decision` 同时订阅 String 和结构化，结构化优先。

#### 8.6 0x0120 sentry_cmd 下行打通
- 新增 `rm_interfaces/msg/referee/SentryCommand.msg`。
- `serial_driver` 新增 `0xB6 HEADER_SENTRY_CMD` 下行包。
- `sentry_decision` 的 `send_stance_command()` 发布 `sentry/command` topic。
