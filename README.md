# capability_mission_scenarios

场景数据与任务配置独立于规划算法仓库维护。当前场景名称：

- `zju2`：原 `capability_mission_planner/tmp/栅格示例/1`，单地图场景
- `myj1`：原 `capability_mission_planner/tmp/栅格示例/2`，七地图场景

目录结构：

```text
maps/zju2/                 zju2 的 Nav2 YAML/PNG
maps/myj1/                 myj1 的多地图 YAML/PNG/关系/传送点
configs/zju2/              zju2 的任务与算法回归配置
configs/myj1.yaml          myj1 的任务与算法配置
```

该仓库不复制规划器源码。先构建 `capability_mission_planner`，再直接把配置路径
传给其 CLI：

```bash
PLANNER=/home/jazzy/cpp/capability_mission_planner/build/capability_mission_planner_cli

for config in configs/zju2/*.yaml; do
  "$PLANNER" "$config"
done

"$PLANNER" configs/myj1.yaml
```

结果默认写入本仓库的 `output/`。每次结果包含 `plan.json`、`summary.txt` 和
每张地图的 `routes_*.png`。配置中的相对地图路径均相对于配置文件所在目录。

如果要运行算法仓库的场景回归测试：

```bash
cmake -S /home/jazzy/cpp/capability_mission_planner \
  -B /home/jazzy/cpp/capability_mission_planner/build \
  -DBUILD_SCENARIO_TESTS=ON \
  -DSCENARIO_ROOT=/home/jazzy/cpp/capability_mission_scenarios
cmake --build /home/jazzy/cpp/capability_mission_planner/build --parallel
ctest --test-dir /home/jazzy/cpp/capability_mission_planner/build --output-on-failure
```
