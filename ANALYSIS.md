# sentry_decision_sample — 哨兵导航调度器

> 最后更新: 2026-07-14 | 目标机器人: 全自动哨兵 (Sentry) | 赛季: RM2026

---

## 1. 定位

`sentry_decision_sample` 是一个**纯导航调度器**。它只回答一个问题：**哨兵下一步往哪走。**

它不参与战斗（自瞄独立运作），不控制姿态（电控负责），不感知敌人。它只从裁判系统读取血量、弹药和前哨站状态，然后决定去巡逻还是回补给区。目标机器人是**全自动哨兵**（满血 400），弹药兑换/复活确认/姿态切换等全部交给电控，本模块只回答"下一步往哪走"。

### 与原版 `sentry_decision` 的关系

```
原版 sentry_decision (1,900 行)   → 完整 FSM，包揽导航+战斗+姿态，保留作参考
精简版 sentry_decision_sample     → 纯导航 FSM，只调度目的地，当前主线
```

### 设计取舍

| 决策模块做 | 决策模块不做 |
|-----------|-------------|
| 血量低(hp<150)→去补给区 | 看到敌人→追击（自瞄独立） |
| 弹药低(≤50)→去补给区 | 被攻击→反击（电控+自瞄） |
| 补满(hp满且弹药≥100)→出去巡逻 | 主动兑换弹药 / 确认复活（电控） |
| 有血有弹→巡逻 | 切换进攻/防御姿态（电控） |
| 前哨存活→前压路线 / 前哨亡→半场防守 | 判断该不该开火（自瞄） |
| 阵亡复活后持续导航回补给区 | 接收雷达/自瞄数据 |

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
② RESUPPLY  ← 已在补给中且未恢复满 → 保持
              退出条件：hp 回满 400 且 ammo ≥ 100（两者都满足才出）
③ RESUPPLY  ← hp < 150 或 ammo ≤ 50 → 进入
④ PATROL    ← 以上都不满足时的默认状态
```

**迟滞设计**：进入用 `hp<150 / ammo≤50`，退出用 `hp满 / ammo≥100`，进出阈值分离防止边界抖动。弹药靠补给区每分钟被动 +100 恢复，一个免费周期即可从 50 补过 100。

**复活兜底**：哨兵阵亡时 `hp=0`，会留在 RESUPPLY；复活后（hp 从 0 恢复但仍 <150）继续保持 RESUPPLY 并持续导航回补给区，直到恢复满。这是"永不放弃"设计的直接结果。

### 4.2 状态切换防振荡 (can_leave_current_state)

- IDLE 可以随时离开
- 进入 IDLE 总是允许
- RESUPPLY 可以立即抢断 PATROL（血/弹不足优先）
- 其他状态间切换需要 `min_ticks_in_state`（默认 4 ticks = 400ms）

### 4.3 状态行为

| 状态 | 导航目标 | 说明 |
|------|---------|------|
| IDLE | 无（取消所有导航） | 裁判断连或比赛未运行时原地等待 |
| PATROL | `patrol` / `patrol_aggressive` 路线循环 | 前哨站存活走 `patrol_aggressive`（前压），被打掉走 `patrol`（我方半场防守）。路线切换时重置路点追踪 |
| RESUPPLY | `supply`（+ 可选 `backup_supply_points` 轮换） | RFID 滑动窗口确认到达后停留恢复；未到达则持续导航，卡住/超时就轮换候选点再回主点，**永不放弃**。满血且弹药≥100 后离开 |

---

## 5. 代码架构

```
sentry_decision_sample/
├── CMakeLists.txt
├── package.xml
├── include/sentry_decision_sample/
│   ├── types.hpp          # 枚举、结构体、阈值
│   ├── context.hpp        # Context: 比赛状态聚合器
│   ├── profile.hpp        # Profile: YAML 加载
│   ├── fsm.hpp            # DecisionFsm: 状态机声明
│   └── decision_node.hpp  # DecisionNode: ROS2 节点声明
├── src/
│   ├── profile.cpp        # YAML 解析
│   ├── fsm.cpp            # FSM 实现
│   └── decision_node.cpp  # 节点: 订阅、action client、fallback
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
| `State` (enum) | IDLE / PATROL / RESUPPLY |
| `NavStatus` (enum) | IDLE / MOVING / ARRIVED / FAILED |
| `Thresholds` | 8 个可配置阈值，全部有默认值 |

### 5.3 context.hpp — 世界模型

无 ROS 依赖，纯 C++ 数据聚合。关键方法：

**裁判相关**：`game_running()`, `remain_time()`, `referee_fresh()`, `hp()`, `max_hp()`, `hp_low()`, `hp_full()`, `ammo()`, `ammo_low()`, `ammo_ok()`, `needs_resupply()`, `resupply_done()`

**场地相关**：`on_supply_pad()`, `outpost_alive()`, `ally_base_hp()`

**导航相关**：`nav_status()`, `nav_failed()`, `goal_reached()`, `set_nav_status()`

**位置相关**：`sentry_x()`, `sentry_y()`, `sentry_pos_valid()`, `set_sentry_position()`

### 5.4 fsm.cpp — 核心 FSM

- **tick()**：select_state → can_leave_current_state → on_exit/on_enter → run_behaviour
- **select_state()**：3 级优先级链（IDLE / RESUPPLY / PATROL），每 tick 重评估
- **behave_patrol()**：按前哨站状态二选一路线（存活前压 / 被打掉半场防守），路线切换时重置路点追踪，卡住跳点
- **behave_resupply()**：RFID 滑动窗口防抖确认到达 → 未到达则持续导航；nav_failed 或单点超时轮换候选点（主点↔备用点循环），**永不放弃**，天然覆盖复活回归

---

## 6. 鲁棒性设计

### 6.1 防振荡

- **HP/弹药迟滞**：进入 hp<150 或 ammo≤50，退出需 hp满且 ammo≥100，进出阈值分离防边界抖动
- **min_ticks_in_state**：PATROL 切换需停留 4 tick（400ms），防止瞬态抖动（RESUPPLY 抢断不受限，血/弹优先）
- **路线切换重置**：巡逻路线切换时重置 `path_idx_`/`goal_sent_`，避免拿新路线索引判断旧目标的到达/超时

### 6.2 超时安全网

| 超时 | 默认值 | 作用 |
|------|--------|------|
| `stuck_timeout_s` | 10s | 单个巡逻点超时→跳过下一个点 |
| `resupply_timeout_s` | 30s | 单个补给点超时→轮换到下一候选点（循环，不放弃） |

> 注：旧版的"总超时后原地放弃"逻辑已移除。RESUPPLY 只要未恢复满就持续导航，确保阵亡复活后一定能回补给区。

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
| `hp_low` | 150 | 血量低于此值 → 进 RESUPPLY |
| `ammo_low` | 50 | 弹药 ≤ 此值 → 进 RESUPPLY |
| `ammo_ok` | 100 | 弹药 ≥ 此值（且血满）→ 退出 RESUPPLY |
| `game_total_time` | 420 | 比赛总时长（秒） |
| `min_ticks_in_state` | 4 | 状态最小停留 tick |
| `stuck_timeout_s` | 10.0 | 路点卡住超时（秒） |
| `resupply_timeout_s` | 30.0 | 单个补给点超时→轮换（秒） |
| `referee_stale_timeout_s` | 3.0 | 裁判数据过期时间（秒） |

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

PATROL 状态每 tick 按**我方前哨站状态**二选一：

```
① outpost_alive（前哨站存活）且 patrol_aggressive 非空 → patrol_aggressive（前压）
② 否则（前哨站被打掉，或未配激进路线）              → patrol（我方半场防守）
```

依据：前哨站存活时己方基地无敌，可放心前压；前哨站被击毁后基地暴露，退回半场防守。

路线切换时会重置路点追踪（`path_idx_`/`goal_sent_`/`goal_arrived_`），从新路线起点重新发目标，避免追错航点。换战术只需修改 YAML，无需重新编译。

> **末局决策未实现**：规则上"双方前哨站均被摧毁 + 基地血量胶着"时靠全队总伤害定胜负，此时该攻该守取决于**敌我基地血量对比**。但当前 `GameRobotHP.msg` 只解析了己方字段（`enemy_base_hp`/`enemy_outpost_hp`/`damage_difference` 未接入），读不到敌方数据，故末局专用逻辑暂不实现，等上游串口驱动补全敌方字段后再做。

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
  -p profile_path:=~/omni_navigation/src/omni_decision_sample/config/profiles/rmuc_red.yaml

# 蓝方
ros2 run sentry_decision_sample sentry_decision_sample_node --ros-args \
  -p profile_path:=~/omni_navigation/src/omni_decision_sample/config/profiles/rmuc_blue.yaml
```

---

## 10. 当前状态

| 项目 | 状态 |
|------|------|
| 状态机 | 3 态（IDLE / PATROL / RESUPPLY） |
| 单元测试 | 26/26 通过 |
| 编译警告 | 0 |
| 死代码 | 0 |
| 已知逻辑缺陷 | 0 |
| 待验证 | Profile 中的坐标需在实车场地标定确认 |
| 未实现（有意） | 末局攻守决策（依赖敌方基地血量，当前链路读不到） |

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
