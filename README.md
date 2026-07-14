# sentry_decision_sample

> 哨兵自主导航调度器。只管一个问题：**下一步往哪走。**

| 仓库 | 分支 | 本地路径 |
|------|------|----------|
| [lmy-lmy-666/sentry_decision](https://github.com/lmy-lmy-666/sentry_decision) | `omni_decision_sample` | `/home/lmy/omni_navigation/src/omni_decision_sample` |

## 三个状态

```
有血有弹      → PATROL   （沿巡逻路线循环走点）
hp<150 或 弹药≤50 → RESUPPLY （回唯一补给区，回满 400 且弹药≥100 再出来，永不放弃）
裁判断连      → IDLE   （原地不动）
```

弹药靠规则里"占领补给区每分钟自动 +100"被动恢复，决策不主动兑换。
补给区是唯一的恢复点，也是阵亡复活后的回归点——RESUPPLY 只要没恢复满就一直尝试导航回去，绝不中途放弃。

## 快速开始

```bash
# 编译
cd ~/omni_navigation
source install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sentry_decision_sample

# 运行（红方）
ros2 run sentry_decision_sample sentry_decision_sample_node --ros-args \
  -p profile_path:=~/omni_navigation/src/omni_decision_sample/config/profiles/rmuc_red.yaml

# 运行测试
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --packages-select sentry_decision_sample
./build/sentry_decision_sample/fsm_test
```

## 换战术

改 YAML 就行，不用重新编译。关键参数：

```yaml
thresholds:
  hp_low: 150       # 血量低于这个 → 去补给
  ammo_low: 50      # 弹药 ≤ 这个 → 去补给
  ammo_ok: 100      # 弹药 ≥ 这个（且血满）→ 结束补给出去巡逻

patrol:               # 默认巡逻路线（前哨站被打掉时用 → 我方半场防守）
  - {x: 3.88, y: 2.67, dwell_s: 10.0}

patrol_aggressive:    # 前哨站存活时用 → 前压
  - {x: 8.0, y: 0.0, dwell_s: 10.0}

supply: {x: -1.02, y: -4.91}   # 唯一补给点（也是恢复/复活回归点）
```

巡逻路线按**我方前哨站状态**二选一：存活走 `patrol_aggressive`（前压），被打掉走 `patrol`（我方半场防守）。

## 不做什么

- 不追敌人（自瞄独立运作）
- 不切换姿态（电控负责）
- 不收雷达/自瞄数据

## 详细文档

见 [ANALYSIS.md](./ANALYSIS.md)
