# bdz1 场景

`03_homogeneous_shared_charger_benchmark.yaml` 用于评估共同充电桩区域下的
同质多机器人任务分配：三台机器人从同一条通道的相邻栅格出发，能力、速度、
净空参数完全相同；12 个任务的能力要求、类别和服务时间完全相同。

该配置关闭 `coordinate_conflicts`，因此输出的负载主要反映静态 A* 路径和任务
分配，不包含 CBS 等待时间。每个 A* 段在给定栅格代价模型下是最短路径；整个
多机器人任务分配使用启发式插入，结果用于验证“接近最短”和负载均衡趋势，不能
宣称严格的全局多旅行商最优。

运行：

```bash
./build/capability_mission_planner_cli \
  /home/jazzy/cpp/capability_mission_scenarios/configs/bdz1/03_homogeneous_shared_charger_benchmark.yaml \
  /home/jazzy/cpp/capability_mission_scenarios/output/bdz1/03_homogeneous_shared_charger_benchmark
```

结果目录包含 `plan.json`、`summary.txt` 和 `routes_bdz1.png`。
