# sentry_decision Profile 坐标说明

> 日期: 2026-07-05

## 需要你在实车/地图上确认的坐标

以下坐标在 profile YAML 中定义，会直接影响哨兵导航行为。坐标使用 map 坐标系（原点为红方补给站附近场地围挡交点，X 轴向蓝方，Y 轴沿场地短边向红方停机坪）。

### 1. 补给区坐标

```yaml
supply: {x: ?, y: ?}
```

- 哨兵 RESUPPLY 时的目的地。
- **必须是己方补给区附近的可达位置**，到达后会靠 RFID 确认是否真的站在补给区上。
- 需要确认的点数：**1 个**（也可配多个 backup 点）。

### 2. 撤退点坐标

```yaml
retreat: {x: ?, y: ?}
```

- 哨兵 RETREAT 时的第一目的地。
- **应该是己方半场最安全的位置**，尽量靠近补给区或基地。
- 需要确认的点数：**1 个**。

### 3. 掩体/安全点坐标

```yaml
safe_cover: {x: ?, y: ?}
```

- EVADE、HARDEN、EVADE_AIR 和 RETREAT 失败时的避难目标。
- **必须是实际可以到达的掩体附近**，不能是空地。
- 需要确认的点数：**1 个**。

### 4. 巡逻路点坐标

```yaml
patrol:
  - {x: ?, y: ?, dwell_s: ?}
  - {x: ?, y: ?, dwell_s: ?}
  ...
```

- PATROL 状态和比赛后期保守巡逻使用。
- 每点需设驻留时间 `dwell_s`。
- **应该覆盖己方半场关键防守区域**，避免靠近规则禁区。
- 需要确认的点数：**3～5 个**。

### 5. 前压路点坐标

```yaml
attack_push:
  - {x: ?, y: ?, dwell_s: ?}
  ...
```

- ATTACK_PUSH 状态使用。
- **必须确保路径不会穿越规则禁区（基地禁区、公路禁区等）**。
- 需要确认的点数：**3～5 个**。

### 6. 防守巡逻路点坐标

```yaml
defend_fallback:
  - {x: ?, y: ?, dwell_s: ?}
  ...
```

- DEFEND/SCOUT 未发现敌人时使用。
- **应是己方半场关键防守位置**。
- 需要确认的点数：**2～4 个**。

### 7. 备用补给点坐标（可选）

```yaml
backup_supply_points:
  - {x: ?, y: ?}
  ...
```

- 主补给点 nav_stuck 或超时后依次尝试。
- 需要确认的点数：**0～3 个**（可不配）。

### 8. 备用撤退点坐标（可选）

```yaml
backup_retreat_points:
  - {x: ?, y: ?}
  ...
```

- 主撤退点 nav_stuck 或超时后依次尝试。
- 需要确认的点数：**0～3 个**（可不配）。

### 9. 比赛后期保守巡逻（可选）

```yaml
late_game_leading:
  fallback_patrol:
    - {x: ?, y: ?, dwell_s: ?}
    ...
```

- 比赛后期（<60s）使用，替代正常 patrol 路线。
- 需要确认的点数：**0～4 个**（可不配）。

---

## 总结清单

| 坐标 | 数量 | 优先级 |
|------|------|--------|
| supply | 1 | 必须 |
| retreat | 1 | 必须 |
| safe_cover | 1 | 必须 |
| patrol | 3～5 | 必须 |
| attack_push | 3～5 | 必须 |
| defend_fallback | 2～4 | 推荐 |
| backup_supply_points | 0～3 | 可选 |
| backup_retreat_points | 0～3 | 可选 |
| late_game fallback_patrol | 0～4 | 可选 |

## 禁区提醒

所有坐标和路线必须避开：
- **基地禁区**（双方禁入）
- **公路禁区**（双方禁入）
- 对方补给禁区（仅对方可进）
- Nav2 costmap 中应预先标记这些禁区为 lethal
