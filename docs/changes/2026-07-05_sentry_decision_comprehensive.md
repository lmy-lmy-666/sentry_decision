# 2026-07-05 sentry_decision 全部改动汇总

## 一、基础稳定性修复

| 修复 | 文件 |
|------|------|
| `Context::now()` 返回真实时间 | `context.hpp` |
| `on_supply_pad()` 合并两个补给 RFID | `context.hpp` |
| 空 route 取模 0 保护 | `fsm.cpp` |
| RETREAT/RESUPPLY 滞后退出（hysteresis） | `fsm.cpp` |
| `min_ticks_in_state` 启用 | `fsm.cpp` |
| stance 5s 冷却 + 去重 + DEFENSIVE 可绕过冷却 | `fsm.cpp` |

## 二、新接口接入

### 2.1 自瞄输入（近距离）

```text
topic: auto_aim_target_pos (std_msgs/String: "x,y,valid,id")
→ parse_auto_aim_target() → EnemyInfo
```

`decision_node.cpp` 新增 `auto_aim_target_topic` 参数。

### 2.2 雷达输入（全局坐标）

```text
topic: radar/enemy_positions (rm_interfaces/EnemyPosition)
→ radar_callback() → 聚合 6 个位置 → EnemyInfo
```

新增 `rm_interfaces/msg/EnemyPosition.msg`。
`decision_node.cpp` 新增订阅 + callback。

### 2.3 motion_state 结构化

```text
topic: motion_manager/motion_state (rm_interfaces/MotionState)
→ motion_state_structured_callback()
```

新增 `rm_interfaces/msg/MotionState.msg`。
`sentry_motion_manager` 新增结构化 publisher。
`sentry_decision` 优先订阅结构化，String 作为 fallback。

### 2.4 0x0120 sentry_cmd 下行

```text
sentry_decision → sentry/command (SentryCommand)
→ serial_driver → 0xB6 → STM32
```

新增 `rm_interfaces/msg/referee/SentryCommand.msg`。
`serial_driver` 新增 `HEADER_SENTRY_CMD = 0xB6` 下行包。
`sentry_decision` 的 `send_stance_command()` 发布到 `sentry/command`。

## 三、FSM 战术改进（8 项）

| 改进 | 效果 |
|------|------|
| ATTACK_PUSH 被攻击立即退出 | `attack_push_allowed()` 增加 `!ctx.under_attack()` |
| RESUPPLY 失败兜底 | backup 耗尽 → hp_critical 保持，否则退出到 PATROL |
| RETREAT backup | nav_stuck/timeout → backup_retreat_points → safe_cover |
| DEFEND/SCOUT 防守巡逻 | 无敌人时使用 `defend_fallback` 路线 |
| late_game 配置启用 | 后期使用 `late_game_leading.disable_attack_push` |
| Combat SubState 超时 | TRACK 20s / ENGAGE 30s → fallback SCOUT |
| motion_state 解析加固 | 词边界检查 + 连续 3 次 idle 才判 FAILED |
| 裁判 stale 参数化 | `referee_stale_timeout_s` 替代硬编码 3.0s |

## 四、Bug 修复（4 个）

| Bug | 修复 |
|-----|------|
| `combat_harden` 计时错误 | `state_entered_s_` → `substate_entered_s_` |
| EnemyInfo 无过期保护 | 新增 `enemy_info_fresh()`，0.5s 过期 |
| RESUPPLY 不防敌 | stay 条件加 `!enemy_detected()` |
| RESUPPLY↔DEFEND 振荡 | enter 条件加 `!enemy_detected()` |

## 五、配套工程改动

| 工程 | 改动 |
|------|------|
| `rm_interfaces` | 新增 `SentryCommand.msg`, `MotionState.msg`, `EnemyPosition.msg` |
| `rm_serial_driver` | 新增 `0xB6` sentry_cmd 下行包 |
| `sentry_motion_manager` | 新增结构化 `MotionState` publisher，依赖 `rm_interfaces` |
| `RM2026_BOF_Radar` | 新增 `ros_publisher.py` 惰性 ROS2 桥接 |
| `bof_26_vision` | 新增结构化 Target publisher（仅哨兵，#ifdef 保护） |

## 六、验证

```text
编译: rm_interfaces + rm_serial_driver + sentry_decision + sentry_motion_manager 全部通过
GTest: sentry_decision 19/19, sentry_motion_manager 13/13
Lint: 7/7
多状态场景: 手动 ROS2 topic 模拟验证通过
```
