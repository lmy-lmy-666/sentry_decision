# sentry_decision — 哨兵导航决策模块

> Sentry26 导航系统核心决策层 | 基于 C++17 FSM | RM2026 赛季

---

## 一、在 Sentry26 中的位置

```text
Sentry26 导航系统
├── rm_serial_driver        ← 串口通信（裁判系统 ↔ STM32）
├── sentry_motion_manager   ← 运动仲裁（Nav2 → cmd_vel）
├── sentry_decision         ← 战术决策（本模块）
│   ├── 订阅裁判/自瞄/雷达/运动状态
│   ├── FSM 决定当前状态（IDLE/PATROL/DEFEND/ATTACK_PUSH/RESUPPLY/RETREAT）
│   └── 输出 Nav2 goal + 姿态指令（sentry/command）
└── Nav2                     ← 路径规划 + 控制
```

决策是导航系统的大脑：**根据比赛态势决定哨兵往哪走、用什么姿态。**

---

## 二、输入源

| 数据 | 来源 | 通信方式 |
|------|------|---------|
| 裁判系统（血量/弹药/热量/时间段） | serial_driver | ROS2 topic `/referee/*` |
| 自瞄目标（近距离） | bof_26_vision | ROS2 topic `auto_aim_target_pos` |
| 雷达敌方坐标（全局） | RM2026_BOF_Radar | ROS2 topic `/radar/enemy_positions` |
| 运动状态 | sentry_motion_manager | topic `motion_manager/state` + `motion_manager/motion_state` |
| 里程计 | Nav2/定位 | ROS2 topic `/odometry` |

---

## 三、输出

| 输出 | 目标 | 说明 |
|------|------|------|
| Nav2 goal | `navigate_to_pose` action | 导航路点 |
| `/goal_pose` | PoseStamped topic | Nav2 宕机时的 fallback |
| 姿态指令 | `sentry/command` → serial_driver → STM32 | 进攻/防御/移动/强化姿态 |
| 远程兑换/复活 | `sentry/command` | 字段预留，待 FSM 驱动 |

---

## 四、核心状态机

### 顶层状态（优先级从高到低，每 tick 重评估）

```
IDLE          ← 比赛未运行 / 裁判数据失效
RETREAT       ← hp < 60（已在 RETREAT 则 hp < 120 退出）
DEFEND(空中)  ← 空中威胁
RESUPPLY      ← 弹药空 / hp < 150（进入/退出有 hysteresis）
DEFEND(地面)  ← 敌方检测 / 被攻击
ATTACK_PUSH   ← 前哨存活 + hp≥180 + 弹药充足 + 不过热 + 不被攻击 + 非后期
PATROL        ← 默认
```

### 战斗子状态

```
SCOUT →(发现敌人)→ TRACK →(距离<3m)→ ENGAGE →(被击中)→ EVADE →(1s安全)→ SCOUT
         ↕                       ↕ 敌人跑远>8m         ↕ 被击中
         追击移动              原地交火                进攻反击
空中威胁 → HARDEN(15s强化) → EVADE_AIR →(无威胁)→ SCOUT
```

### 关键行为

| 状态 | 移动 | 姿态 | 说明 |
|------|------|------|------|
| IDLE | 停 | — | 安全模式 |
| PATROL | 巡逻路线 | MOBILITY | 后期切 fallback_patrol |
| DEFEND/SCOUT | defend_fallback 路线 | DEFENSIVE | 防守巡逻搜索 |
| DEFEND/TRACK | **向敌人追击** | DEFENSIVE | 追到交火距离 |
| DEFEND/ENGAGE | 停住交火 | OFFENSIVE | 近距离射击 |
| DEFEND/EVADE | 不动 | **OFFENSIVE** | 被打立刻反击 |
| DEFEND/HARDEN | 撤向 safe_cover | ENHANCED_DEFENSIVE | 空中威胁 |
| ATTACK_PUSH | attack_push 路线 | OFFENSIVE | 前压 |
| RESUPPLY | 去补给区 | MOBILITY | 多级失败兜底+冷却 |
| RETREAT | 去补给区 | DEFENSIVE | 多级撤退兜底 |

---

## 五、稳定性和鲁棒性

- **State hysteresis**：进入/退出用不同阈值，防止状态振荡
- **min_ticks_in_state**：最小停留 400ms 防瞬态抖动
- **RESUPPLY 冷却**：补给失败后 15s 不重试，防死循环振荡
- **stance 冷却**：5s 间隔 + 去重检测
- **数据 stale 保护**：裁判 3s / 敌人 1s 无数据自动安全兜底
- **导航容错**：Nav2 宕机自动 fallback，卡住跳点，cancel 完整清理
- **自瞄+雷达融合**：自瞄管检测，雷达管坐标，各不覆盖
- **所有数据访问**：`std::optional` 保护，null 返回安全默认值

---

## 六、与雷达站的关系

### 雷达站路径

```text
/home/lmy/RM2026_BOF_Radar
```

### 数据流

```text
雷达 → SDR 接收 → 串口 → 裁判系统（不变）
雷达 → ROS2 /radar/enemy_positions → sentry_decision 收到全局敌方坐标
```

### 决策端的雷达数据使用

- `nearest_distance` + `nearest_x`/`nearest_y`：TRACK 状态追击导航
- `aerial_threat`：触发 HARDEN/EVADE_AIR 防空子状态
- `count`：未来可用于多敌人威胁评估

---

## 七、与自瞄的关系

- 自瞄通过 `auto_aim_target_pos`（`std_msgs/String: "x,y,valid,id"`）提供近距离检测
- 决策端解析字符串获取 `detected` 和 `nearest_distance`
- 自瞄坐标信息不覆盖雷达的全局坐标（数据融合保护）
- 结构化 `Target` 消息接口已预留，自瞄端开关未打开

---

## 八、代码端已完成的改进

### 2026-07-07 稳定性修复
- RESUPPLY 防死循环振荡（15s 冷却 + 到达重置 + 弹药空绕过）
- EVADE 切进攻姿态反击（不再无效跑掩体）
- TRACK 主动追击（雷达坐标驱动导航）
- EnemyInfo 过期时间可配（0.5s → 1.0s）
- cancel_nav fallback 标志位清理
- 自瞄不覆盖雷达坐标
- 导航卡住时 goal 清理（combat_track / combat_evade_air）

### 历史改进
- 基础稳定性（now()、RFID、空 route、hysteresis、姿态冷却去重）
- 自瞄 String + 雷达 EnemyPosition 双源输入
- motion_state 结构化 topic + String fallback
- 0x0120 sentry_cmd 下行打通（ROS 端）
- 战术改进（ATTACK_PUSH 被攻即退、RESUPPLY 失败兜底、RETREAT 备用路线、DEFEND 防守巡逻、late_game 策略、Combat SubState 超时）
- Nav2 fallback 到达检测（odom 距离）

---

## 九、未解决的问题

| 问题 | 影响 | 需要谁 |
|------|------|--------|
| STM32 未解析 0xB6 | 姿态切换只发 ROS，不到裁判系统 | 电控 |
| 0x020D 未接入 | 脱战状态、当前姿态不可知 | 电控 + serial_driver |
| 0x020C 未接入（可选） | 空中威胁、双倍易伤无来源 | 电控 + serial_driver |
| Profile 坐标未填 | 补给点、撤退点、巡逻路线为空 | 实车地图确认后填入 |
| `io::ROS2 ros2;` 未传 true | sentry.yaml 已配但代码未读 config → 自瞄结构化消息开关未生效 | 自瞄同学 |

---

## 十、编译与测试

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm_interfaces radar_msgs sentry_decision

# 测试
export LD_LIBRARY_PATH=~/Sentry26/install/lib:~/Sentry26/install/rm_interfaces/lib:~/Sentry26/install/radar_msgs/lib:$LD_LIBRARY_PATH
~/Sentry26/build/sentry_decision/fsm_test
```

```text
GTest: 30/30
编译: 全部通过
```

## 十一、运行

```bash
source /opt/ros/jazzy/setup.bash
source ~/Sentry26/install/setup.bash

# 红方
ros2 run sentry_decision sentry_decision_node --ros-args \
  -p profile_path:=~/Sentry26/src/sentry_decision/config/profiles/rmuc_red.yaml

# 蓝方
ros2 run sentry_decision sentry_decision_node --ros-args \
  -p profile_path:=~/Sentry26/src/sentry_decision/config/profiles/rmuc_blue.yaml
```

## 十二、更多文档

| 文档 | 内容 |
|------|------|
| `ANALYSIS.md` | 完整架构说明、修改日志、参数表、**待实战验证风险点** |
| `docs/interface_audit.md` | 接口审计与接入方向 |
| `docs/profile_coordinates.md` | Profile 坐标填写说明 |
| `docs/测试指南.md` | 测试说明 |
| `docs/changes/` | 历次修改日志 |
| `docs/剩余待完成事项.md` | 待完成事项清单 |
