# sentry_decision_sample — 哨兵导航调度器

> 最后更新: 2026-07-09 | 目标机器人: 哨兵 (Sentry) | 赛季: RM2026

---

## 1. 定位

`sentry_decision_sample` 是一个**纯导航调度器**。它只回答一个问题：**哨兵下一步往哪走。**

它不参与战斗（自瞄独立运作），不控制姿态（电控负责），不感知敌人。它只从裁判系统读取血量和弹药，然后决定去巡逻、去补给、还是撤退。

### 与原版 `sentry_decision` 的关系

```
原版 sentry_decision (1,900 行)   → 完整 FSM，包揽导航+战斗+姿态，保留作参考
精简版 sentry_decision_sample     → 纯导航 FSM，只调度目的地，当前主线
```

### 设计取舍

| 决策模块做 | 决策模块不做 |
|-----------|-------------|
| 血量低→去补给区 | 看到敌人→追击（自瞄独立） |
| 弹药空→去补给区 | 被攻击→反击（电控+自瞄） |
| 致命血量→回补给区 | 切换进攻/防御姿态（电控） |
| 有血有弹→巡逻 | 判断该不该开火（自瞄） |
| 前哨存活→激进巡逻路线 | 接收雷达/自瞄数据 |

---

## 2. 输入源

| 数据源 | 话题 | 用途 |
|--------|------|------|
| 裁判系统-比赛状态 | `referee/game_status` | 判断比赛是否运行、剩余时间 |
| 裁判系统-机器人状态 | `referee/robot_status` | 当前血量、弹药、最大血量 |
| 裁判系统-RFID | `referee/rfidStatus` | 是否在补给区（确认到达） |
| 裁判系统-全体血量 | `referee/all_robot_hp` | 前哨站是否存活、己方基地血量 |
| 里程计 | `odometry` | 己方位置（Nav2 fallback 到达检测用） |

共 **5 个订阅**。

## 3. 输出

| 输出 | 目标 | 说明 |
|------|------|------|
| Nav2 goal | `navigate_to_pose` action | 导航目标点（主通道） |
| `/goal_pose` | PoseStamped topic | Nav2 不可用时的 fallback |

仅 **1 个输出通道**（Nav2 action，降级到 topic）。

---

## 4. 顶层状态机

### 4.1 优先级链（每 tick 重评估，命中第一个满足的即返回）

```
优先级从高到低：

① IDLE      ← 裁判数据失效（超过 3 秒）或比赛未运行
② RETREAT   ← hp < 60。已在 RETREAT 则 hp < 120 才保持（hysteresis）
③ RESUPPLY  ← 已在补给中且仍需补。弹药空时不退出。
              ├─ 补给耗尽 & hp ≥ 120 → 退出
              └─ 未耗尽 & (弹药空 or hp < max_hp) → 保持
④ RESUPPLY  ← 弹药空 or hp < 150。冷却期 15s 内不重试（弹药空绕过）
⑤ PATROL    ← 以上都不满足时的默认状态
```

### 4.2 状态切换防振荡 (can_leave_current_state)

- IDLE 可以随时离开
- 进入 IDLE / RETREAT 总是允许（安全优先）
- RESUPPLY 可以抢断除 RETREAT 外的任何状态
- 其他状态间切换需要 `min_ticks_in_state`（默认 4 ticks = 400ms）

### 4.3 状态行为

| 状态 | 导航目标 | 说明 |
|------|---------|------|
| IDLE | 无（取消所有导航） | 裁判断连或比赛未运行时原地等待 |
| PATROL | `patrol` 路线循环 | 前哨存活自动切 `patrol_aggressive`，最后 60 秒自动切 `patrol_late` |
| RESUPPLY | `supply` → `backup_supply_points[N]` | RFID 滑动窗口确认到达后停留回血，满血满弹后离开 |
| RETREAT | `supply`（重试直到总超时） | 致命血量时直接回补给区。补给区在己方基地（敌方禁区），永远可达 |

---

## 5. 代码架构

```
sentry_decision_sample/
├── CMakeLists.txt
├── package.xml
├── include/sentry_decision_sample/
│   ├── types.hpp          # 枚举、结构体、阈值（77 行）
│   ├── context.hpp        # Context: 比赛状态聚合器（180 行）
│   ├── profile.hpp        # Profile: YAML 加载（38 行）
│   ├── fsm.hpp            # DecisionFsm: 状态机声明（90 行）
│   └── decision_node.hpp  # DecisionNode: ROS2 节点声明（82 行）
├── src/
│   ├── profile.cpp        # YAML 解析（95 行）
│   ├── fsm.cpp            # FSM 实现（364 行）
│   └── decision_node.cpp  # 节点: 订阅、action client、fallback（270 行）
├── config/profiles/
│   ├── rmuc_red.yaml / rmuc_blue.yaml
├── launch/
│   └── sentry_decision_sample_launch.py
└── test/
    └── fsm_test.cpp       # 26 个单元测试
```

### 5.1 依赖层次（单向，上层不依赖下层）

```
types.hpp  ← 纯数据，零依赖
profile.hpp → types.hpp
context.hpp → types.hpp + rm_interfaces
fsm.hpp → context.hpp + profile.hpp
decision_node.hpp → context.hpp + fsm.hpp + ROS2
```

- `Context`：零 ROS 依赖，可脱离 ROS 单独测试
- `DecisionFsm`：零 ROS 依赖，通过 `std::function` 回调与节点解耦
- `DecisionNode`：唯一的 ROS 层，负责订阅、发布、Action 客户端

### 5.2 types.hpp — 核心类型

| 类型 | 说明 |
|------|------|
| `Waypoint {x, y, dwell_s}` | 导航目标点 |
| `Route = vector<Waypoint>` | 路点序列 |
| `State` (enum) | IDLE / PATROL / RESUPPLY / RETREAT |
| `NavStatus` (enum) | IDLE / MOVING / ARRIVED / FAILED |
| `Thresholds` | 14 个可配置阈值，全部有默认值 |

### 5.3 context.hpp — 世界模型

无 ROS 依赖，纯 C++ 数据聚合。关键方法：

**裁判相关**：`game_running()`, `remain_time()`, `late_game()`, `referee_fresh()`, `hp()`, `max_hp()`, `hp_critical()`, `hp_low()`, `ammo()`, `ammo_empty()`, `needs_resupply()`

**场地相关**：`on_supply_pad()`, `outpost_alive()`, `ally_base_hp()`

**导航相关**：`nav_status()`, `nav_failed()`, `goal_reached()`, `set_nav_status()`

**位置相关**：`sentry_x()`, `sentry_y()`, `sentry_pos_valid()`, `set_sentry_position()`

### 5.4 fsm.cpp — 核心 FSM

- **tick()**：select_state → can_leave_current_state → on_exit/on_enter → run_behaviour
- **select_state()**：5 级优先级链，每 tick 重评估
- **behave_patrol()**：巡逻路线循环，前哨存活/比赛后期自动切换战术路线
- **behave_resupply()**：RFID 滑动窗口防抖 → 总超时 → nav_failed 跳备用 → 单点超时 → 耗尽冷却
- **behave_retreat()**：持续导航到补给区，总超时兜底，卡住重试（Nav2 重新规划路径）

---

## 6. 鲁棒性设计

### 6.1 防振荡

- **RETREAT hysteresis**：进入 hp<60，退出 hp≥120（60HP 窗口）
- **RESUPPLY 冷却**：补给全部失败后 15s 不重试（弹药空绕过），防止 PATROL↔RESUPPLY 死循环
- **min_ticks_in_state**：非安全状态切换需停留 4 tick（400ms），防止瞬态抖动
- **RESUPPLY→RETREAT 直通**：安全优先，从补给状态可直接切入撤退

### 6.2 超时安全网

| 超时 | 默认值 | 作用 |
|------|--------|------|
| `stuck_timeout_s` | 30s | 单个巡逻点超时→跳过 |
| `resupply_timeout_s` | 30s | 单个补给点超时→切备用 |
| `resupply_total_timeout_s` | 120s | 整条补给链超时→耗尽冷却 |
| `retreat_timeout_s` | 30s | 撤退单次超时→重试 |
| `retreat_total_timeout_s` | 60s | 整次撤退超时→原地停留 |

### 6.3 数据安全

- **裁判 stale 超时**：3s 无数据→IDLE，取消所有导航
- **所有数据访问**：`std::optional` 保护，null 时返回安全默认值
- **RFID 滑动窗口防抖**：最近 5 帧中 ≥3 帧在补给区才确认，容忍偶发信号丢失
- **线程安全**：依赖 `SingleThreadedExecutor`，所有回调和 timer 在同一线程串行

### 6.4 导航容错

- **Nav2 action 可用**：走 Action 协议（feedback/result 回调，goal_id 比对防 stale）
- **Nav2 action 不可用**：自动降级到 PoseStamped topic + odom 距离到达检测
- **导航失败**：`nav_failed()` → 跳下一个路点或重试
- **cancel_nav**：始终 `async_cancel_all_goals`，fallback 模式下正确清理标志位

---

## 7. 可配置参数

### YAML thresholds（默认值，可在 profile YAML 中覆盖）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hp_low` | 150 | 低血量阈值（低于此值去补给） |
| `hp_critical` | 60 | 致命血量阈值（低于此值撤退） |
| `hp_critical_exit` | 120 | 撤退退出滞后值 |
| `ammo_min` | 1 | 弹药耗尽阈值 |
| `game_total_time` | 420 | 比赛总时长（秒） |
| `min_ticks_in_state` | 4 | 状态最小停留 tick |
| `stuck_timeout_s` | 30.0 | 路点卡住超时（秒） |
| `resupply_timeout_s` | 30.0 | 单个补给点超时（秒） |
| `resupply_total_timeout_s` | 120.0 | 补给链总超时（秒） |
| `retreat_timeout_s` | 30.0 | 撤退单点超时（秒） |
| `retreat_total_timeout_s` | 60.0 | 撤退总超时（秒） |
| `referee_stale_timeout_s` | 3.0 | 裁判数据过期时间（秒） |
| `supply_retry_cooldown_s` | 15.0 | 补给失败后冷却时间（秒） |

### ROS2 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `profile_path` | 必填 | YAML 战术文件路径 |
| `tick_frequency` | 10.0 Hz | FSM tick 频率 |
| `goal_topic` | `/goal_pose` | fallback goal 话题 |
| `nav_action_name` | `navigate_to_pose` | Nav2 action 名称 |
| `goal_frame` | `map` | goal 坐标帧 |
| `goal_reached_distance_tolerance` | 0.25 m | 到达判定距离 |

---

## 8. Patrol 战术路线切换规则

PATROL 状态每 tick 自动选择巡逻路线，优先级如下：

```
① late_game（剩余 < 60s）且 patrol_late 非空 → 使用 patrol_late
② outpost_alive（前哨站存活）且 patrol_aggressive 非空 → 使用 patrol_aggressive
③ 以上都不满足 → 使用默认 patrol
```

换战术只需修改 YAML，无需重新编译。

---

## 9. 编译与运行

```bash
cd ~/omni_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sentry_decision_sample

# 单元测试
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --packages-select sentry_decision_sample
./build/sentry_decision_sample/fsm_test

# 红方
ros2 run sentry_decision_sample sentry_decision_sample_node --ros-args \
  -p profile_path:=~/omni_navigation/src/sentry_decision_sample/config/profiles/rmuc_red.yaml

# 蓝方
ros2 run sentry_decision_sample sentry_decision_sample_node --ros-args \
  -p profile_path:=~/omni_navigation/src/sentry_decision_sample/config/profiles/rmuc_blue.yaml
```

---

## 10. 当前状态

| 项目 | 状态 |
|------|------|
| 代码行数 | ~950 行 |
| 单元测试 | 26/26 通过 |
| 编译警告 | 0 |
| 死代码 | 0 |
| 已知逻辑缺陷 | 0 |
| 待验证 | Profile 中的坐标需在实车场地确认 |

---

## 11. 与外部模块的边界

```
┌─────────────────────────────────────────────────┐
│                 sentry_decision_sample           │
│                                                 │
│  裁判(血量/弹药) ──→ Context ──→ FSM ──→ Nav2    │
│                                                 │
│  只管一个问题: "下一步往哪走"                      │
└─────────────────────────────────────────────────┘
         │                              │
         │ 不需要                        │ 需要
         ▼                              ▼
┌─────────────────┐          ┌─────────────────┐
│  自瞄 (视觉组)   │          │  Nav2 导航      │
│  独立运作        │          │  路径规划+执行   │
│  看到人自动锁    │          │                 │
└─────────────────┘          └─────────────────┘
         │
         │ 不需要（电控组）
         ▼
┌─────────────────┐
│  姿态切换        │
│  电控根据自瞄    │
│  状态自行切换    │
└─────────────────┘
```
