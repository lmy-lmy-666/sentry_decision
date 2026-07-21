# omni_behavior_sample

> 哨兵自主导航调度器 —— **行为树（Behavior Tree）版**。
> 功能与 `omni_decision_sample`（有限状态机版）**完全一致**，只是把决策的控制流从手写状态机换成了一棵行为树。

| 对照 | FSM 版 | 本包（BT 版） |
|------|--------|--------------|
| 包名 | `omni_decision_sample` | `omni_behavior_sample` |
| 决策引擎 | `DecisionFsm`（`select_state` 优先级链 + `switch` 分派） | `BtDecision`（一棵行为树，每 tick 从根重评估） |
| BT 库依赖 | 无 | **无**（自带零依赖轻量内核 `behavior_tree.hpp`，不需装 `BehaviorTree.CPP`） |
| 行为 | —— | 与 FSM 版逐条对齐，66 个单测全部平移通过 |

两版共用同一套领域类型（`types` / `context` / `profile` / `arrival_tracker`，仅命名空间不同），所以世界模型、阈值、坐标标定、和电控的接法都**一模一样**。换用哪个包，机器人的实际行为不变。

## 五种行为（和 FSM 版相同）

```
比赛开局(可选) → OPENING_STRIKE （先去打点位, 让自瞄摧毁敌方前哨站, 停留一段后转正常, 每局一次）
遇到起伏路段    → BUMP_TRAVERSE  （舵轮纯直线开环冲过波浪地形, 绕过 Nav2, 穿越中不可打断）
有血有弹      → PATROL   （沿巡逻路线循环走点）
hp<150 或 弹药≤50 → RESUPPLY （回唯一补给区，回满 400 且弹药≥100 再出来，永不放弃）
裁判断连      → IDLE   （原地不动）
```

优先级：IDLE > BUMP_TRAVERSE(穿越中) > RESUPPLY > OPENING_STRIKE > PATROL。

> 在 BT 版里，这五个不再是"状态变量"，而是**这一 tick 里哪条树枝跑了**的标签（用于日志/telemetry 与 FSM 版对齐）。树每 tick 从根重新评估，天然是"响应式"的。

## 状态机是怎么变成树的

FSM 的 `tick()` 做三件事：① 按优先级选一个候选状态 → ② 带防抖地提交 → ③ 跑该状态的行为。BT 版保留这**同样的三步**，每步都是一棵子树：

```
Root (ReactiveSequence，每 tick 顺序跑完 4 步)
 ├── SelectionFallback   (ReactiveFallback = 优先级仲裁)   → 定 candidate
 │     ├── [裁判断连/比赛未跑]           → IDLE           （最高优先级）
 │     ├── [BUMP 穿越/收尾进行中]        → BUMP_TRAVERSE  （只有 IDLE 能抢占）
 │     └── [业务逻辑 + 起伏段入口判定]    → RESUPPLY / OPENING / PATROL / BUMP
 ├── Commit              (防抖门 = can_leave_current_state + on_enter)
 ├── ArrivalDetect       (锁存目标到达 = run_behaviour 前处理)
 └── DispatchFallback    (跑已提交树枝的行为)
       ├── [==IDLE]           → behave_idle
       ├── [==OPENING_STRIKE] → behave_opening_strike
       ├── [==PATROL]         → behave_patrol
       ├── [==RESUPPLY]       → behave_resupply
       └── [==BUMP_TRAVERSE]  → BumpPhaseTree（5 相 ReactiveFallback 子树）
```

关键映射：

| FSM 概念 | BT 表达 |
|----------|---------|
| `select_state` 优先级链（首个命中者胜） | `ReactiveFallback`：从头逐子节点 tick，首个非 FAILURE 者胜，高优先级枝可抢占低优先级正在 RUNNING 的枝 |
| `can_leave_current_state` 防抖（min_ticks / RESUPPLY 立即抢占 / BUMP 穿越中不可打断） | `Commit` 动作节点，逐条复刻同样的判断 |
| `switch(state_)` 行为分派 | `DispatchFallback`：每个树枝 = `Condition(是否本状态) → Action(跑行为)` |
| BUMP 的 5 相内嵌子状态机 + `for(;;)` 相位推进 | 独立的 `BumpPhaseTree`（5 相各一枝的 ReactiveFallback）；相位处理器返回 `SUCCESS`=本 tick 继续重评估（FSM 的 `continue`），`RUNNING`=结束本 tick（FSM 的 `return`），由 `behave_bump` 循环重 tick |

内核（`behavior_tree.hpp`）提供的词汇：`Sequence` / `ReactiveSequence` / `Fallback(Selector)` / `ReactiveFallback` / `Inverter` / `ForceSuccess` / `Condition` / `Action`。纯头文件、无 ROS、无第三方 BT 库，任何 ROS 发行版（本包在 **Jazzy** 上验证）都能直接编。

## 过起伏路段（波浪地形）

机制与约束和 FSM 版完全相同：与目标分处起伏段两侧且靠近入口时自动接管 —— Nav2 开到入口对正朝向 → cancel Nav2 → 舵轮纯直线恒速开环冲 → 到出口 x 硬停 → 交还。恒速不减速、盲走兜底（TF 丢失保持上一帧速）、超时反向退回并禁用该段、方向自动判定。在 BT 版里这就是 `BumpPhaseTree` 那棵 5 相子树。

## 快速开始

```bash
# 编译
cd ~/omni_navigation
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select omni_behavior_sample
source install/setup.bash

# 运行（红方 / 蓝方）
ros2 launch omni_behavior_sample omni_behavior_sample_launch.py profile:=rmuc_red.yaml
ros2 launch omni_behavior_sample omni_behavior_sample_launch.py profile:=rmuc_blue.yaml

# 也可用完整路径
ros2 launch omni_behavior_sample omni_behavior_sample_launch.py \
  profile:=~/omni_navigation/src/omni_behavior_sample/config/profiles/rmuc_red.yaml

# 运行测试
colcon test --packages-select omni_behavior_sample --event-handlers console_direct+
#   bt_decision_test      决策行为 (32)  ← 与 FSM 版 fsm_test 逐条对齐
#   bump_test             过起伏路段 (12)
#   arrival_tracker_test  Nav2 到达判定 (11)
#   behavior_tree_test    BT 内核语义 (11)  ← BT 版新增
```

## 换战术

改 YAML 就行，不用重新编译。所有字段、含义、坐标标定方法与 FSM 版**完全相同**（`config/profiles/*.yaml` 直接复用）。关键参数：

```yaml
thresholds:
  hp_low: 150       # 血量低于这个 → 去补给
  ammo_low: 50      # 弹药 ≤ 这个 → 去补给
  ammo_ok: 100      # 弹药 ≥ 这个（且血满）→ 结束补给出去巡逻
  opening_strike_duration_s: 90.0   # 开局打点位停留时长(秒)

opening_strike: {x: 2.0, y: 0.0}   # 开局打前哨站位; 删掉此项=关闭该功能
patrol:               # 默认巡逻路线（前哨站被打掉时用 → 我方半场防守）
  - {x: 3.88, y: 2.67, dwell_s: 10.0}
patrol_aggressive:    # 我方前哨站存活时用 → 前压
  - {x: 8.0, y: 0.0, dwell_s: 10.0}
supply: {x: -1.02, y: -4.91}   # 唯一补给点（也是恢复/复活回归点）

bump_segments:        # 起伏段列表; 空=不启用。entry/exit 必须同 y(沿x轴直线)
  - entry: {x: 0.7,   y: -7.086}
    exit:  {x: 5.458, y: -7.086}
    yaw: 0.0
```

> ⚠️ 起伏段坐标必须实车逐点标定，红蓝分别标。`bump_segments: []`（空）即关闭该功能。

## 和电控怎么接

与 FSM 版一致，链路不变：

```
决策/Nav2 → /cmd_vel_chassis (Twist) → rm_serial_driver 打包 lx/ly/az → 电控舵轮
```

## 与 FSM 版的关系

本包是 `omni_decision_sample` 的**等价重写**，用于对比"同一套哨兵决策，状态机 vs 行为树"两种实现范式。行为完全一致（单测逐条平移通过），可任选其一部署：

- 想要**最直白、单文件读完**的控制流 → FSM 版
- 想要**可组合、可视化、易扩展新枝**的控制流 → BT 版（本包）

`docs/` 下的设计文档（状态流转图、坐标标定、实车测试指南等）与 FSM 版共享，概念完全通用。
