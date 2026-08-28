# 单地图覆盖场景

所有场景使用 `maps/zju2`，点位均位于同一个连通自由区域，并保留至少
3 个栅格的障碍净空。

| 配置 | 机器人/任务 | 主要覆盖内容 |
|---|---:|---|
| `01_single_generalist.yaml` | 1 / 8 | 单机多点、多任务类型、优先级、同点任务合并 |
| `02_four_specialists.yaml` | 4 / 10 | 灭火、拍照、热成像、搬运专用机器人及复合能力任务 |
| `03_overlapping_capabilities.yaml` | 3 / 12 | 能力重叠下的任务竞争、负载均衡与总距离权衡 |
| `04_more_robots_than_tasks.yaml` | 6 / 3 | 机器人多于任务、空闲机器人及严格能力匹配 |
| `05_many_tasks_few_robots.yaml` | 2 / 16 | 任务明显多于机器人、多点连续访问和返航 |
| `06_shared_sites.yaml` | 4 / 12 | 4 个点位各含多个属性任务、同点停靠合并 |

运行全部场景：

```bash
for config in configs/zju2/*.yaml; do
  /home/jazzy/cpp/capability_mission_planner/build/capability_mission_planner_cli "$config"
done
```

也可以通过 `ctest --test-dir build --output-on-failure` 执行自动回归。
