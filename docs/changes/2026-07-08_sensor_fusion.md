# 2026-07-08 自瞄+雷达双源融合 + Bug修复

## 自瞄+雷达双源融合（读时融合）

### 问题
两个回调各自构造完整 `EnemyInfo` 后调用 `context_.update()` 全量覆盖。后到达的来源覆盖先到达的，造成数据丢失：
- 自瞄覆盖雷达 → 丢失全局坐标，无法追击
- 雷达覆盖自瞄 → 丢失快速检测结果
- `detected` 竞态：自瞄丢锁会清除雷达的检测

### 方案
读时融合（read-time fusion）：每个源独立维护自己的检测标记和时间戳，读取时 OR 融合。

### 改动

**context.hpp** — 新增独立标记和独立更新方法：

```cpp
// 自瞄独立标记
bool aim_detected_{false};
double aim_stamp_{0.0};

// 雷达独立标记
bool radar_has_target_{false};
double radar_stamp_{0.0};

// 各自写自己的，互不触碰
void update_enemy_from_aim(bool detected, double distance);  // 只写 aim_detected_
void update_enemy_from_radar(double x, double y, int count, bool aerial); // 只写 radar_has_target_

// 读时 OR 融合，无竞态
bool enemy_detected() const {
    return (aim_detected_ && fresh) || (radar_has_target_ && fresh);
}
```

**decision_node.cpp** — 两个回调改用融合接口：
- `auto_aim_target_callback` → `context_.update_enemy_from_aim()`
- `radar_callback` → `context_.update_enemy_from_radar()`
- `parse_auto_aim_target` 改为返回 `(bool& detected, double& distance)`
- 雷达回调修正 `hypot` 参考系（到哨兵距离）和 `min_dist` 追踪（取最近敌人）

### 效果
- 自瞄看到敌人 + 雷达有坐标 → `enemy_detected()=true`, `nearest_x/y` 有效，可追击
- 自瞄丢锁 + 雷达还在 → `enemy_detected()` 仍 true，不丢敌情
- 雷达断 + 自瞄还在 → 同理
- 旧接口 `update(EnemyInfo&)` 保留兼容单元测试，同步双标记

---

## Bug修复

### #1 HARDEN→SCOUT goal_sent_ 未重置
空中威胁消失回搜索时 `goal_sent_` 仍为 true（safe_cover goal 未到达），导致 SCOUT 巡逻不发新 goal。修复：切 SCOUT 时 `goal_sent_ = false`。

### #2 EVADE OFFENSIVE 被冷却丢弃
`combat_evade` 中 `switch_stance(OFFENSIVE)` 不在冷却白名单中，若刚切过 DEFENSIVE（5s内）则被静默丢弃。修复：`switch_stance(OFFENSIVE, force=true)` 强制绕过冷却。

### #3 雷达最近敌人选错（融合改造时引入的回归）
`dist < 999.0` 取最后一个有效 slot，不是最近的。修复：恢复 `min_dist` 追踪。

### #4 雷达 hypot 参考系错误（预存问题）
`hypot(s.x, s.y)` 算到地图原点的距离。修复：`hypot(s.x - sentry_x, s.y - sentry_y)` 算到哨兵距离。

## 串口 stance 下行

- `protocol.yaml`: `reserved` → `stance`，注释改为"姿态指令: 1进攻 2防御 3移动..."
- `generate.py` 重新生成 `packet.hpp` / `navigation_auto.h`
- `sendControlPacket`: `pkt.stance = current_stance_`

## omni_navigation 适配

- RFID topic: `referee/rfid_status` → `referee/rfidStatus`
- rm_interfaces 补 `MotionState.msg` + `SentryCommand.msg`
- serial_driver 加 `sentry/command` 订阅
- 删除无效 motion_state 订阅

## 验证

- 编译: ✅
- 单元测试: ✅ 30/30
- 双源融合: ✅ 自瞄丢锁不覆盖雷达，雷达断不丢自瞄检测
