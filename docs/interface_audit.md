# sentry_decision 接口审计

> 日期: 2026-07-07  
> 目标: 评估 `/home/lmy/Sentry26/src/sentry_decision` 要达到比赛可用还缺哪些输入/输出闭环。  
> 审计范围: `/home/lmy/Sentry26/src` 下的决策、串口、接口、motion manager、导航 bringup 相关文件。

---

## 1. 总体结论

当前 `sentry_decision` 的 FSM 骨架和测试基础已经完整，顶层 6 状态：

- `IDLE`
- `PATROL`
- `DEFEND`（含 6 个战斗子状态）
- `ATTACK_PUSH`
- `RESUPPLY`
- `RETREAT`

代码端已闭环，剩余依赖外部：

1. **EnemyInfo 已有自瞄+雷达双源输入**：自瞄做近距离检测，雷达提供全局坐标用于追击。已通过 bag 回放验证数据流。
2. **SentryInfo 没有真实来源**：没有 0x020D/姿态/脱战/强化剩余时间接口。
3. **0x0120 下行已打通（ROS 端）**：`sentry/command` → `serial_driver` → `0xB6`。STM32 解析待配合。
4. **motion_state 双通道**：结构化 + String fallback，已加固解析。
5. **fallback goal 已有 odom 到达检测**：Nav2 不可用时自动切换。
6. **RESUPPLY 冷却 + EVADE 反击 + TRACK 追击 + cancel_nav 清理**：全部已实现。

优先级建议：

```text
P0: STM32 解析 0xB6
P1: SentryInfo/0x020D 接入
P2: 自瞄结构化 Target、Profile 坐标确认、仿真闭环验证
```

---

## 2. 当前已有接口

### 2.1 rm_interfaces 消息

路径：`/home/lmy/Sentry26/src/rm_interfaces/msg`

已有 referee 消息：

- `referee/GameStatus.msg`
- `referee/RobotStatus.msg`
- `referee/RfidStatus.msg`
- `referee/GameRobotHP.msg`

已有 vision 消息：

- `vision/Armor.msg`
- `vision/Armors.msg`
- `vision/Target.msg`

#### GameStatus.msg

字段：

```text
behavior_state
游戏阶段 game_progress
剩余时间 stage_remain_time
```

可用于：

- `IDLE` / `RUNNING` 判断
- `late_game()`

#### RobotStatus.msg

字段包含：

```text
robot_id
robot_level
current_hp
maximum_hp
shooter_barrel_cooling_value
shooter_barrel_heat_limit
add_ok
shooter_17mm_1_barrel_heat
robot_pos
armor_id
hp_deduction_reason
projectile_allowance_17mm
remaining_gold_coin
is_hp_deduced
```

可用于：

- HP / ammo / heat
- 被击中来源
- 当前机器人位置 `robot_pos`
- 金币 `remaining_gold_coin`

注意：当前 `sentry_decision::Context` 已使用其中部分字段，但还没有充分利用：

- `robot_level`
- `shooter_barrel_heat_limit`
- `shooter_barrel_cooling_value`
- `robot_pos`
- `remaining_gold_coin`

#### RfidStatus.msg

当前实际补给区字段为：

```text
friendly_supply_zone_non_exchange
friendly_supply_zone_exchange
```

没有单独的 `friendly_supply_zone` 字段。  
因此 `Context::on_supply_pad()` 合并这两个字段是符合当前接口定义的。

其他可用增益点字段：

- `base_gain_point`
- `friendly_fortress_gain_point`
- `friendly_outpost_gain_point`
- `center_gain_point`
- 多个高地/地形跨越增益点

目前决策只用了：

- 补给区
- 基地增益点

其余增益点策略尚未使用。

#### GameRobotHP.msg

当前字段只有己方 HP：

```text
ally_1_robot_hp
ally_2_robot_hp
ally_3_robot_hp
ally_4_robot_hp
ally_7_robot_hp
ally_outpost_hp
ally_base_hp
```

重要限制：

- 没有 `enemy_base_hp`
- 没有 `enemy_outpost_hp`
- 没有敌方机器人 HP

这意味着当前无法直接判断：

- 后期领先/落后
- 对方前哨站是否已掉
- 对方基地血量优势

如果要做 late_game leading/trailing，需要扩展底层 HP 消息或另接完整裁判系统 0x0003。

#### vision/Target.msg

字段：

```text
header
tracking
id
armors_num
position
velocity
yaw
v_yaw
radius_1
radius_2
dz
```

这可以作为 `EnemyInfo` 的近期来源。

可转换为：

```cpp
EnemyInfo.detected = tracking;
EnemyInfo.nearest_distance = norm(position);
EnemyInfo.count = tracking ? 1 : 0;
```

局限：

- `id` 是字符串，需要映射为机器人类型。
- 无直接 `near_base/near_outpost`，需要结合 map 坐标和自车/目标坐标系。
- 若 `position` 是 base_link 坐标，只能做距离/近战判断；若是 map 坐标，才能做区域威胁判断。

---

## 3. 当前串口 driver 审计

路径：

- `serial/serial_driver/src/rm_serial_driver.cpp`
- `serial/serial_driver/include/rm_serial_driver/packet.hpp`
- `serial/serial_driver/protocol/protocol.yaml`

### 3.1 已接收包

当前自定义串口协议包：

```text
0xA1 HEADER_IMU
0xA2 HEADER_STATUS
0xA3 HEADER_HP
```

#### 0xA2 STATUS

字段：

```text
game_progress
stage_remain_time
current_hp
projectile_allowance_17mm
rfid_base
```

映射到 ROS：

```text
referee/game_status
referee/robot_status
referee/rfid_status
```

注意：`rfid_base` 被映射到：

```cpp
RfidStatus.friendly_supply_zone_non_exchange
```

这只是兼容旧行为树命名，不等价于完整 RM2026 0x0209 RFID 状态。

#### 0xA3 HP

字段：

```text
ally_1_robot_hp
ally_2_robot_hp
ally_3_robot_hp
ally_4_robot_hp
ally_7_robot_hp
ally_outpost_hp
ally_base_hp
```

只包含己方 HP。

### 3.2 已发送包

当前只发送：

```text
0xB5 HEADER_NAV_TX
vel_x
vel_y
vel_w
```

即导航速度指令。

### 3.3 缺失内容

当前 serial_driver 没有：

- `0x0120` sentry_cmd 下行
- `0x020D` 哨兵自主决策信息同步上行
- `0x020C` 雷达标记进度上行
- 完整 `0x0209` RFID 位图
- 完整 `0x0003` 双方 HP
- 底盘能量 / buff / 允许兑换数量等信息

因此当前 `sentry_decision` 中：

- `send_stance_command()` 无法真正下发。
- `SentryInfo` 没法从真实裁判系统填充。
- `EnemyInfo.double_vulnerability_active` 没真实来源。
- `under_aerial_attack()` 没真实雷达标记来源。

---

## 4. motion_manager 接口审计

路径：

- `sentry_motion_manager/src/motion_manager_node.cpp`
- `sentry_motion_manager/config/motion_manager.yaml`

当前状态：

- 原有 String topic `motion_manager/state` 保留兼容。
- 新增结构化 topic `motion_manager/motion_state`（`rm_interfaces/msg/MotionState`），含 enum 常量：
  - `mode`: IDLE/NAVIGATION/RECOVERY/...
  - `recovery_phase`: IDLE/STRAIGHT_RELEASE/LOW_CURVATURE_RELEASE/ARC_ESCAPE/SUCCEEDED/FAILED
  - `output_enabled`, `emergency_stop`, `has_fresh_command`
- `sentry_decision` 同时订阅两者，结构化 topic 优先生效（String 作为 fallback）。

**已解决**：字符串格式变动不再导致静默失效。

短期保留 String，但增强解析容错和日志。  
中期新增结构化消息，例如：

```text
MotionState.msg
uint8 mode
uint8 selected_source
uint8 recovery_phase
bool output_enabled
bool emergency_stop
bool has_fresh_command
float32 recovery_projected_progress_m
float32 distance_to_goal
```

---

## 5. EnemyInfo 接入审计

当前状态：

- `sentry_decision` 已经在决策端兼容自瞄当前输出。
- 新增参数：`auto_aim_target_topic`，默认 `auto_aim_target_pos`。
- 订阅类型：`std_msgs/String`。
- 数据格式：`x,y,valid,id`。
- 转换结果：更新内部 `EnemyInfo`。

当前转换逻辑：

```cpp
EnemyInfo.detected = valid > 0.5 && id > 0.5;
EnemyInfo.nearest_distance = detected ? hypot(x, y) : 999.0;
EnemyInfo.count = detected ? 1 : 0;
```

验证结果：

通过手动发布模拟数据：

```text
auto_aim_target_pos: "1.0,0.0,1,3"
```

并同时模拟 referee 输入，已观察到 FSM 正常进入：

```text
DEFEND/TRACK
DEFEND/ENGAGE
```

因此，近距离自瞄目标到 `EnemyInfo` 的链路已经打通。

局限：

- 当前仍是字符串解析，不如结构化消息稳。
- 没有 `header/frame_id`。
- 当前自瞄输出的 `x/y` 来自 `armor.xyz_in_gimbal[0/1]`，更接近云台/自瞄内部坐标，不是 map 全局坐标。
- 只能可靠用于 `detected/distance/count`，不适合直接判断 `enemy_near_base/enemy_near_outpost`。
- 空中威胁和双倍易伤仍没有真实来源。

中期建议：

如果允许修改自瞄工程，保留旧 `auto_aim_target_pos` 的同时，新增结构化 topic：

```text
/auto_aim/target
rm_interfaces/msg/vision/Target
```

这样 `sentry_decision` 后续可以优先订阅结构化 topic，字符串兼容作为 fallback。

### 路径 B：雷达站全局坐标

已接入。雷达站（`RM2026_BOF_Radar`）新增 `ros_publisher.py`，发布：

```text
radar/enemy_positions
rm_interfaces/msg/EnemyPosition (x, y, robot_type)
```

`sentry_decision` 新增订阅 `radar/enemy_positions`，在 `radar_callback` 中聚合 6 个敌方位置为 `EnemyInfo`。

优势：
- 全局 map 坐标（cm→m），可直接判断 `enemy_near_base / enemy_near_outpost`。
- 包含空中机器人类型（robot_type=4），可标记 `aerial_threat`。
- 雷达站侧惰性初始化，不影响雷达原有功能。

剩余：`enemy_near_base / enemy_near_outpost` 阈值计算需要 profile YAML 提供 base/outpost 参考坐标。

---

## 6. SentryInfo / 0x020D 接入审计

当前情况：

- `SentryInfo` 只有结构体和测试入口。
- serial_driver 没有 0x020D 解析。
- rm_interfaces 没有对应 `SentryInfo.msg`。

建议新增消息：

```text
rm_interfaces/msg/referee/SentryInfo.msg
```

建议字段：

```text
bool disengaged
uint8 current_stance
bool stance_enhanced
float32 offensive_remaining_s
float32 defensive_remaining_s
float32 mobility_remaining_s
float32 enhanced_offensive_remaining_s
float32 enhanced_defensive_remaining_s
float32 enhanced_mobility_remaining_s
bool energy_mechanism_activatable
```

然后 serial_driver 或 referee_bridge 发布：

```text
referee/sentry_info
```

`sentry_decision` 订阅后转换为内部 `SentryInfo`。

---

## 7. 0x0120 下行审计

当前状态：

- `sentry_decision::DecisionNode::send_stance_command()` 已不再是 STUB，现已发布 `sentry/command` (SentryCommand)。
- 新增 `rm_interfaces/msg/referee/SentryCommand.msg`，覆盖 0x0120 全部字段。
- `serial_driver` 新增 `HEADER_SENTRY_CMD = 0xB6` 下行包，订阅 `sentry/command` 并通过串口发送给 STM32。
- 已通过 ROS2 topic 模拟验证整条链路。

剩余依赖：

- STM32 侧需要解析 `0xB6` 并转换为 RM2026 `0x0120 sentry_cmd` 发送。
- 当前 `sentry/command` 只填充 `stance_command` 字段，其余字段（远程兑换、复活等）待 FSM 后续驱动。

### 新增消息

```text
rm_interfaces/msg/referee/SentryCommand.msg
```

字段：

```text
bool confirm_revive
bool confirm_instant_revive
uint16 projectile_exchange_amount
uint8 remote_projectile_exchange_count
uint8 remote_hp_exchange_count
uint8 stance_command
bool activate_energy_mechanism
```

### 新增串口下行包

```text
0xB6 HEADER_SENTRY_CMD (15 bytes, ros_to_stm32, ~10Hz)
```

定义在 `protocol.yaml`，结构体在 `packet.hpp`，处理函数在 `rm_serial_driver.cpp`。


---

## 8. Nav2 / fallback 审计

`sentry_decision` 已经使用 Nav2 `NavigateToPose` action：

- goal accepted → `MOVING`
- feedback distance 小于阈值 → `ARRIVED`
- result succeeded → `ARRIVED`
- rejected/aborted/unknown → `FAILED`

**已解决**：

- fallback 模式已加入 odom 距离检测（`on_tick` 中 `hypot(dx, dy) <= 0.25m` → `ARRIVED`）。
- `cancel_nav()` 在 action server 不可用时也清理 `nav_goals_active_` 和 `fallback_goal_active_`。

---

## 9. 上场前必须闭环清单

### 已完成
- [x] EnemyInfo 自瞄+雷达双源输入，自瞄管检测、雷达管坐标
- [x] `SentryCommand.msg` + `sentry/command` → `serial_driver` → `0xB6` 下行
- [x] RESUPPLY 防死循环振荡（15s 冷却 + 到达重置 + 弹药空绕过）
- [x] EVADE 切进攻姿态反击
- [x] TRACK 雷达坐标驱动追击（EnemyInfo 新增 nearest_x/y，Context 新增 sentry_x/y）
- [x] ATTACK_PUSH 被攻击时退出
- [x] RESUPPLY 补给全部超时后退出兜底
- [x] RETREAT stuck/timeout 依次尝试 backup_retreat_points → safe_cover
- [x] DEFEND/SCOUT 使用 defend_fallback 防守巡逻
- [x] late_game 使用 profile late_game_leading 配置
- [x] Combat SubState 超时保护（TRACK 10s / ENGAGE 30s）
- [x] motion_state 解析加固（词边界 + 连续 idle 确认）
- [x] 裁判 stale timeout 参数化（referee_stale_timeout_s，默认 3.0s）
- [x] motion_manager 结构化 MotionState topic
- [x] 雷达站 ROS2 桥接：radar/enemy_positions
- [x] EnemyInfo 过期可配（enemy_stale_timeout_s，默认 1.0s）
- [x] cancel_nav fallback 标志位清理
- [x] Nav2 fallback odom 到达检测
- [x] 裁判数据流验证（bag 回放）
- [x] 雷达数据流验证（bag 回放）
- [x] 单元测试 30/30 通过

### 必须完成
- [ ] STM32 侧解析 `0xB6` 并转换为 RM2026 `0x0120 sentry_cmd` 发送（电控配合）。
- [ ] 接入或新增 `SentryInfo` / 0x020D，用于脱战、当前姿态、强化剩余时间。
- [ ] 确认 Nav2 action 模式可用，减少 fallback 依赖。
- [ ] 明确规则禁区在 costmap 层是否已标 lethal。

### 强烈建议完成
- [ ] 自瞄侧新增结构化 `rm_interfaces/msg/vision/Target` topic 作为 EnemyInfo 中期来源。

---

## 10. 推荐下一步开发顺序

### Step 1：接自瞄 Target → EnemyInfo

这是最短路径，可以马上让 `DEFEND/TRACK/ENGAGE` 真正工作。

### Step 2：设计 SentryCommand.msg + serial_driver 下行

先实现姿态切换，后续再加兑换/复活。

### Step 3：新增 SentryInfo.msg + serial_driver 上行解析

让姿态、脱战、强化剩余时间闭环。

### Step 4：补 RETREAT/RESUPPLY 失败恢复

防止上场时补给/撤退卡死。

### Step 5：做 late_game 和规则禁区检查

进入战术优化阶段。
