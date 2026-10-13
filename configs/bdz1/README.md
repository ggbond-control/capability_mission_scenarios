# bdz1 场景

`03_homogeneous_shared_charger_benchmark.yaml` 用于评估共同充电桩区域下的
同质多机器人任务分配：三台机器人从同一条通道的相邻栅格出发，能力、速度、
净空参数完全相同；12 个任务的能力要求、类别和服务时间完全相同。

该配置开启 `coordinate_conflicts` 和 `export.animation`。离线规划器先尝试优先级
调度，失败后才运行 CBS；`plan.json` 的 `coordination` 和 GIF 标题会显示实际方法。
`maximum_load_seconds` / `total_load_seconds` 是静态路线负载；实际执行结束时间
见完整 `timed_schedule`，协调等待单独记录。每个 A* 段在给定栅格代价模型下是最短路径；整个
多机器人任务分配使用启发式插入，结果用于验证“接近最短”和负载均衡趋势，不能
宣称严格的全局多旅行商最优。

运行：

```bash
./build/capability_mission_planner_cli \
  /home/jazzy/cpp/capability_mission_scenarios/configs/bdz1/03_homogeneous_shared_charger_benchmark.yaml \
  /home/jazzy/cpp/capability_mission_scenarios/output/bdz1/03_homogeneous_shared_charger_benchmark
```

结果目录包含 `plan.json`、`summary.txt`、`routes_bdz1.png` 和 `routes_overall.gif`。
GIF 默认以约 20 倍速播放，显示三台机器人同步运动、任务停留和协调等待；
所有机器人完成后停留 1 秒，再循环播放。可在 `export` 中调整帧率、倍率、帧数上限
和画面宽度。构建规划器前需安装 giflib (`sudo apt install libgif-dev`)。

当前任务分配中 `charger-b` 空闲，另两台机器人无需协调等待，实际方法是
`prioritized`，所以该配置的动画没有 CBS 等待效果。真正触发 CBS 的验证演示可用：

```bash
/home/jazzy/cpp/capability_mission_planner/build/plan_animation_tests \
  /home/jazzy/cpp/capability_mission_scenarios/output/cbs_wait_demo
```

演示中一台机器人先在起点作业，另一台需要经过该位置；CBS 插入等待后再放行。

## 04 / 05 / 06 协调优化对照

2026-10-13 按原配置测试，成功模式重复三次。文件名中的 60/80 不等于不同点位数：
04、05、06 实际分别为 50/50/52 个不同位置、50/60/80 个原子任务。

| 配置 | 原版 → 当前协调计算时间 | 实际完工时间 | 总等待 | 局部修复 |
|---|---:|---:|---:|---|
| 04 | 0.0495 → 0.0501 s | 1004.7 s，未变 | 1.2 s，未变 | 不触发 |
| 05 | 1.2723 → 0.0814 s | 1008.0 s，未变 | 100.7 s，未变 | 候选未接受 |
| 06 | 0.0507 → 0.0573 s | 1024.7 s，未变 | 0 s | 不触发 |

05 的收益在计算开销和内存，峰值 RSS 约 162 → 78 MB；没有证据表明这三个配置的执行
时间因局部修复得到改善。04/06 原本几乎没有协调等待，计算收益不显著。
本轮没有修改原配置的任务、坐标、速度或机器人尺寸。原配置未设置 footprint 半径，默认 0，
安全余量为 0，故安全结论不能直接外推到半径 0.25 m 的实体机器人。

真实完整轨迹/GIF 复核使用只调整导出选项的副本，结果全部通过，位于
`output/bdz1/extended_benchmarks/04_...` 至 `06_...`。复核检查任务唯一分配、能力、服务时长、
返航、障碍净空、扫掠、资源互斥与 GIF 全帧解码；06 没有等待光圈是正常结果。
原始对照在 `output/bdz1/extended_benchmarks/results.json`，完整报告在算法仓库
`docs/coordination_extended_validation_zh.md`。
