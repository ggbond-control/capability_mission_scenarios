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
