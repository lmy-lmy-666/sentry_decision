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
| 运动状态 | sentry_motion_manager | ROS2 topic `motion_manager/state` + `motion_manager/motion_state` |
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

```text
优先级从高到低（每 tick 重评估）：

IDLE          ← 比赛未运行 / 裁判数据失效
RETREAT       ← 血量 < 60（退出需 > 120，防止抖动）
DEFEND        ← 空中威胁 / 敌方检测 / 被攻击
RESUPPLY      ← 弹药空 / 血量 < 150（退出需 > 180，敌人在时不让路防止振荡）
DEFEND        ← 敌方检测
ATTACK_PUSH   ← 前哨站存活 + 血量弹药充足 + 不过热 + 不被攻击 + 后期不冒进
PATROL        ← 默认
```

---

## 五、与雷达站的关系

### 雷达站路径

```text
/home/lmy/RM2026_BOF_Radar
```

### 改了雷达站什么

| 文件 | 改动 | 说明 |
|------|------|------|
| `ros_publisher.py` | **新建** | ROS2 桥接模块，惰性初始化 |
| `radar_msgs/` | **新建** | 自包含消息包（EnemyPosition.msg），不依赖 Sentry26 |
| `main.py` | +10 行 | 创建 ROS publisher，传入 flowgraph |
| `flowgraph.py` | +6 行 | 透传 ros_pub 参数 |
| `gr_blocks/broadcast_sink.py` | +5 行 | `_on_0x0A01` 末尾调用 ROS 发布 |

**所有改动都是加，原有代码一行没删。** 雷达原有的 SDR 接收、串口通信、GUI、频率切换完全不受影响。

### 效果

```text
改之前：雷达 → 串口 → 裁判系统，ROS 端拿不到敌方坐标
改之后：雷达 → 串口 → 裁判系统（不变）
        雷达 → ROS2 /radar/enemy_positions → sentry_decision 收到全局敌方坐标
```

**决策端能知道敌人在哪了。**

---

## 六、与自瞄的关系

### 自瞄路径

```text
/home/lmy/bof_26_vision
```

### 改了自瞄什么

| 文件 | 改动 | 说明 |
|------|------|------|
| `io/ros2/publish2nav.hpp/cpp` | +Target publisher 开关（#ifdef 保护，默认 false） | 不影响其他兵种 |
| `io/ros2/ros2.hpp/cpp` | 透传开关参数 | 默认 false |
| `configs/sentry.yaml` | +1 行 `enable_target_msg: true` | 仅哨兵配置 |
| `tests/sp_examples/CMakeLists.txt` | 条件查找 rm_interfaces | 没找到就跳过，不影响编译 |

### 效果

```text
改之前：自瞄只发 String "x,y,valid,id" → 决策手动解析
改之后：底层 publish2nav 已具备发结构化 Target 的能力（#ifdef 默认关闭）
```

### 开关状态

底层代码开关已加好，但**哨兵实际入口未打开**。

```text
sentry.yaml 写了 enable_target_msg: true     ← 配置文件已配
io::ROS2 ros2;                                  ← C++ 入口未传 true，开关没开
```

所以当前哨兵实际运行时**不会多发结构化 Target topic**，只发原有的 String topic。  
决策端通过 String 解析正常工作，不受影响。

打开方法：自瞄同学找到哨兵实际的 C++ 入口文件，把 `io::ROS2 ros2;` 改成 `io::ROS2 ros2(true);` 即可。

---

## 七、代码端已完成的改进

- 基础稳定性修复（now()、RFID、空 route、hysteresis、姿态冷却去重）
- 自瞄 String 输入接入
- 雷达 EnemyPosition 输入接入
- motion_state 结构化 topic（String fallback 保留）
- 0x0120 下行 sentry_cmd（ROS 端已通，STM32 待配合）
- FSM 战术改进：ATTACK_PUSH 被攻即退、RESUPPLY 失败兜底、RETREAT 备用路线、DEFEND 防守巡逻、late_game 策略、Combat SubState 超时
- 4 个 Bug 修复（harden 计时、EnemyInfo 过期、RESUPPLY 不防敌、RESUPPLY↔DEFEND 振荡）
- Nav2 fallback 到达检测（odom 距离判断）

---

## 八、未解决的问题

| 问题 | 影响 | 需要谁 |
|------|------|--------|
| STM32 未解析 0xB6 | 姿态切换只发 ROS，不到裁判系统 | 电控 |
| 0x020D 未接入 | 脱战状态、当前姿态不可知 | 电控 + serial_driver |
| 0x020C 未接入（可选） | 空中威胁、双倍易伤无来源 | 电控 + serial_driver |
| Profile 坐标未填 | 补给点、撤退点、巡逻路线为空 | 实车地图确认后填入 |
| `io::ROS2 ros2;` 未传 true | sentry.yaml 已配但代码未读 config → 开关未生效 | 自瞄同学 |

---

## 九、编译与测试

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ~/Sentry26/src --packages-select rm_interfaces radar_msgs sentry_decision

# 测试
source ~/Sentry26/install/setup.bash
ctest --test-dir ~/Sentry26/build/sentry_decision
```

```text
GTest: 19/19
Lint: 7/7
编译: 全部通过
```

## 十、运行

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

---

## 十一、更多文档

| 文档 | 内容 |
|------|------|
| `ANALYSIS.md` | 架构说明与历史问题记录 |
| `docs/interface_audit.md` | 接口审计与接入方向 |
| `docs/profile_coordinates.md` | Profile 坐标填写说明 |
| `docs/剩余待完成事项.md` | 待完成事项清单 |
| `docs/changes/` | 历次修改日志 |
