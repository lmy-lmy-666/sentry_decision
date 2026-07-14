# sentry_decision_sample

> 哨兵自主导航调度器。只管一个问题：**下一步往哪走。**

| 仓库 | 分支 | 本地路径 |
|------|------|----------|
| [lmy-lmy-666/sentry_decision](https://github.com/lmy-lmy-666/sentry_decision) | `omni_decision_sample` | `/home/lmy/omni_navigation/src/omni_decision_sample` |

## 四个状态

```
有血有弹 → PATROL   （沿巡逻路线循环走点）
弹药空/hp<150 → RESUPPLY （去补给区，回满 400 再出来）
hp<60 → RETREAT  （立刻回补给区）
裁判断连 → IDLE   （原地不动）
```

## 快速开始

```bash
# 编译
cd ~/omni_navigation
source install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sentry_decision_sample

# 运行（红方）
ros2 run sentry_decision_sample sentry_decision_sample_node --ros-args \
  -p profile_path:=~/omni_navigation/src/sentry_decision_sample/config/profiles/rmuc_red.yaml

# 运行测试
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --packages-select sentry_decision_sample
./build/sentry_decision_sample/fsm_test
```

## 换战术

改 YAML 就行，不用重新编译。关键参数：

```yaml
thresholds:
  hp_low: 150       # 低于这个去补给
  hp_critical: 60   # 低于这个撤退

patrol:             # 巡逻路线
  - {x: 3.88, y: 2.67, dwell_s: 10.0}

supply: {x: -1.02, y: -4.91}   # 补给点（也是撤退目的地）
```

## 不做什么

- 不追敌人（自瞄独立运作）
- 不切换姿态（电控负责）
- 不收雷达/自瞄数据

## 详细文档

见 [ANALYSIS.md](./ANALYSIS.md)
