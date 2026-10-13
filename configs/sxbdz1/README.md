# sxbdz1 复杂任务验证

`01_six_robots_80_sites_100_tasks.yaml` 用于验证新增地图上的任务分配、带实体尺寸的
多机时间协调、返航和总体 GIF 导出。地图 `map_000` 为 1564×1472，分辨率 0.1 m，
原点 `[-6.6, -11.7, 0]`；关系表只关联 ROOT 与该地图，过渡表没有通道记录。
因此此用例验证单地图行为，不包含楼梯、电梯或跨层切换。

## 场景复杂度

- 6 台机器人，6 种不同的三能力组合；基础能力为 fire、camera、thermal、delivery、gas。
- 80 个不同的任务点、100 个原子任务、21 种任务类别。
- 单能力任务 20 个、双能力任务 40 个、三能力任务 40 个。
- 34 个高优先级任务，服务时长为 2–12 秒；额外复核任务为 3–8 秒。
- 20 组同点任务使用唯一匹配的三能力要求，验证合并停靠和服务时间累计。
- 机器人速度为 0.7/0.8/0.9 m/s，实体半径 0.25 m，安全余量 0.05 m，完成后返航。
- 两机中心距离需大于 0.6 m；障碍净空需至少 0.3 m。
- 时间步长 0.2 s，保守降采样因子 4；动画上限 600 帧、画面宽度 1600。

点位由 `scripts/generate_sxbdz1_benchmark.py` 确定性生成：先筛选净空至少 0.4 m
的栅格，再选取保守 4×4 粗栅格最大的四连通区域，通过最远点采样分散选点。
配置中的位置使用地图局部米制坐标 `local_xy`。地图及其 YAML 保持原样。

## 运行和复核

在本仓库根目录运行（规划器需已构建，GIF 支持需安装 giflib）：

```bash
PLANNER=/home/jazzy/cpp/capability_mission_planner/build/capability_mission_planner_cli
"$PLANNER" configs/sxbdz1/01_six_robots_80_sites_100_tasks.yaml \
  > output/sxbdz1_run.log
python3 scripts/verify_sxbdz1_benchmark.py --run-log output/sxbdz1_run.log
```

`output/` 若尚未存在，先运行 `mkdir -p output`。
Python 验证和点位生成脚本需要 numpy、OpenCV、PyYAML；GIF 解码另需 Pillow。
验证脚本不重新求解，读取导出的完整时间表和原地图，检查：

1. 每个任务恰好分配一次，能力匹配，6 台机器人均有任务；
2. 同点任务合并、任务位置、服务时长和服务期间停靠位置；
3. 起始位置、最终返航位置、所有时间步完整且严格递增；
4. 每个轨迹点及空间段的障碍净空；
5. 每一机器人对、每一时间步的实体距离、扫掠距离和共享边/过渡资源；
6. 等待事件与实际停止推进的时间步一致；
7. 去掉协调等待的基线存在冲突，协调后的轨迹无冲突；
8. GIF 全部帧可解码，尺寸、帧数、循环、最终停留和地图中的等待光圈正确。

算法仓库配置 `BUILD_SCENARIO_TESTS=ON` 后，也注册了回归用例：

```bash
ctest --test-dir /home/jazzy/cpp/capability_mission_planner/build \
  --output-on-failure -R '^single_map_sxbdz1_complex_mission$'
```

重新生成完全相同的点位和任务配置：

```bash
python3 scripts/generate_sxbdz1_benchmark.py
```

该命令会覆盖本用例 YAML，手工修改后应先保存自己的版本。

## 原版本的已验证结果

2026-10-12 实测（本机 Release 构建）：

| 项目 | 结果 |
|---|---:|
| 完成分配的原子任务 | 100/100 |
| 空闲机器人 | 0 |
| 合并停靠点 | 20 |
| 检查的时间步 / 机器人对 | 9339 / 15 |
| 去掉协调等待后的不安全“机器人对×时间步” | 200 |
| 协调后不安全“机器人对×时间步” | 0 |
| 最小机间扫掠距离 | 0.715542 m，要求 > 0.6 m |
| 最小路线障碍净空 | 0.477501 m，要求 ≥ 0.3 m |
| 实际任务结束时间 | 1867.6 s |
| 各机器人的累计协调等待之和 | 1505.8 s |
| 任务规划耗时 | 17.22 s |
| PNG/JSON/GIF 导出耗时 | 7.07 s |
| 总耗时 / 峰值 RSS | 24.30 s / 612 MB |
| GIF | 1600×1210，600 帧，59.44 s，约 3.0 MiB |

| 机器人 | 原子任务 / 停靠点 | 执行结束时间 | 协调等待 |
|---|---:|---:|---:|
| fire-camera-gas | 12 / 8 | 1105.4 s | 0 s |
| camera-thermal-delivery | 18 / 14 | 1129.0 s | 134.0 s |
| fire-thermal-delivery | 23 / 20 | 1089.0 s | 0 s |
| fire-delivery-gas | 14 / 11 | 1361.2 s | 414.2 s |
| camera-thermal-gas | 18 / 15 | 776.0 s | 96.2 s |
| camera-delivery-gas | 15 / 12 | 1867.6 s | 861.4 s |

实际调度方法为 `prioritized`，`cbs_started=false`：优先级调度已消解本场景冲突，
没有触发 CBS 回退。原有 `plan_animation_tests` 的真实 CBS 回退用例仍验证通过。
本结果证明此复杂配置可用，不证明所有点位/任务组合均能求解或达到全局最优。

原版本主要性能开销是时间协调（约 16.7 s），最后一台机器人存在较长等待。这与按固定
机器人优先顺序协调有关；若后续目标是减少总执行时间，应评估不同优先顺序和等待
感知的任务分配。静态 `maximum_load_seconds=1105.4` 不包括协调等待，不能替代
实际结束时间 1867.6 s。保守因子 4 对应约 0.4 m 的任务级路径分辨率；实际控制仍
应使用 Nav2 的实时地图和完整 footprint。

输出位于 `output/sxbdz1/01_six_robots_80_sites_100_tasks/`：

- `plan.json`、`summary.txt`：任务分配、静态负载、导航检查点、等待事件和完整时间表。
- `routes_map_000.png`：静态路线。
- `routes_overall.gif`：真实时间表的同步回放；任务用输入顺序编号，同点任务如 `16/81`。
- `animation_preview.png`：动画中间帧，便于检查地图尺寸、任务标注和等待光圈。
- `validation.json`、`validation.txt`：可重复生成的校验结果。
- `run.log`：本次 CLI 原始日志和性能数据。

GIF 为最终调度动画，不是 CBS 搜索树动画。设置目标 30 倍速，由于 600 帧上限，
本次实际约 32 倍速；285 帧含等待光圈。地图面板已适配纵向地图，单图最大面板
高度提高到 1100 像素，避免 100 个完整任务名称遮挡地图。

## 区间搜索与局部修复后的结果

2026-10-13，规划器增加完整扫掠检查、压缩预约、安全时间区间搜索和一次有界局部修复后，
使用原配置重新执行并独立复核。机器人、任务、能力要求、服务时长、地图、安全距离和降采样
参数均与上述基线相同；100/100 任务、20 个合并停靠及返航均通过检查。

| 项目 | 原版本 | 当前结果 |
|---|---:|---:|
| 实际执行总完工时间 | 1867.6 s | 1363.8 s（减少 27.0%） |
| 累计协调等待 | 1505.8 s | 652.6 s（减少 56.7%） |
| camera-delivery-gas 等待 | 861.4 s | 3.6 s |
| 协调后不安全机器人对×时间步 | 0 | 0 |
| 最小扫掠距离 / 障碍净空 | 0.715542 / 0.477501 m | 相同 |
| 单独计时协调，3 次中位数 | 原测约 16 s | 约 0.19 s |
| 当前 CLI 总耗时 / 峰值 RSS，含 GIF | 原测约 24.3 s / 612 MB | 7.83 s / 117.7 MB |

只重排 camera-delivery-gas 及两个主要阻塞者 camera-thermal-delivery、fire-delivery-gas，
其他三台机器人的时间表保持固定。修复额外展开 7016 个节点，额外计算约 0.08–0.10 秒。
前两个阻塞者分别多等待 2.0 和 2.6 秒，整体等待仍明显降低。
当前方法仍标记为 `prioritized`，JSON 的 `coordination.local_repair.accepted=true`；
未触发 CBS。

输出目录中的 `plan.json`、GIF、预览、日志和 validation 已更新。当前总体 GIF 为
1600×1210、476 帧、约 46.47 秒，其中 196 帧含等待光圈；前述 600 帧数据属于原版本。
动画全部可解码，完整时间表共 6820 个时间步，15 对机器人的独立几何与资源检查全部通过。

默认一次修复上限为 3 台机器人、0.2 秒、20000 个展开节点。无改善、不可行或预算耗尽时保留
原时间表，可通过 `planner.traversal.coordination_local_repair: false` 关闭。
在 19 个配置/顺序组合以及额外 3 个 100 任务压力变体中均未出现执行指标退化；
两个增加服务时长的变体也改善了完工时间，无改善的变体会丢弃候选。
详细实现、分阶段性能和复现脚本见算法仓库
`docs/coordination_optimization_validation_zh.md` 与 `test/benchmark_coordination.py`。

## 更复杂与更广覆盖的扩展场景

`scripts/generate_sxbdz1_extended_benchmarks.py` 新增以下配置，保留原 01 和地图不变。
任务类别均为 21 种，基础能力要求包含 1–3 种组合，服务时间为 2–20 秒，仍含同点复核、
高优先级任务及返航。实体半径/安全余量、时间步长和降采样参数与 01 相同。

| 配置 | 机器人 / 不同点 / 任务 | 部署 | 当前结果 |
|---|---:|---|---|
| 02_six_robots_160_sites_220_tasks | 6 / 160 / 220 | 内部起点 | CBS 512 节点上限失败 |
| 03_twelve_robots_240_sites_360_tasks | 12 / 240 / 360 | 内部起点 | CBS 512 节点上限失败 |
| 04_twelve_robots_240_sites_360_tasks_two_zones | 12 / 240 / 360 | 两区内部起点 | CBS 512 节点上限失败 |
| 05_twenty_robots_320_sites_480_tasks_ten_zones | 20 / 320 / 480 | 十区，主区内部起点 | CBS 512 节点上限失败 |
| 06_six_robots_160_sites_220_tasks_dispersed | 6 / 160 / 220 | 分散起点 | 成功，1699.6 s |
| 07_twelve_robots_240_sites_360_tasks_dispersed | 12 / 240 / 360 | 分散起点 | 成功，1680.2 → 1485.4 s |
| 08_twelve_robots_240_sites_360_tasks_two_zones_dispersed | 12 / 240 / 360 | 两区分别部署 | 成功，1815.4 s |
| 09_twenty_robots_320_sites_480_tasks_ten_zones_dispersed | 20 / 320 / 480 | 十区分别部署 | 成功，2205.2 s |

两区/十区配置使用 `zone_XX` 服务区域资格限制任务所属机器人，避免要求跨越不连通区域。
十区按采样净空 0.4 m 划分；按实际规划净空 0.3 m，它们属于 4 个四连通分量，部分服务区可以
连通，区域资格仍限定岗位任务范围。不能把十区当作十个物理上完全隔离的区域。
十区部署涉及的区域占生成器保守可通行面积的 83.3%，原 01 为 54.6%；该比例不是实际巡视
面积。部署区域内含任务点的 8 m 网格比例约 91.2%，所有筛选格中心距离最近任务点不超过
3.2 m（欧氏距离）。覆盖图与完整度量在 `output/sxbdz1/extended_benchmarks/`。

四个成功场景均完成真实 JSON/GIF 复核，最小机间扫掠距离 0.715542 m，最小障碍净空
0.464859–0.477501 m；所有任务、服务和返航正确。07 的局部修复减少完工时间 11.6%、总等待
8.7%；其他三个场景的候选没有被接受。不能宣称所有场景都能从局部修复获益。

修改前版本在 06 输出一处实际扫掠冲突，在 07–09 达到 CBS 上限失败；02–05 的失败同样存在
于旧版本或在 40 秒 / 2 GiB 地址空间限制内未完成。保留这些失败配置作为求解边界实验。
详细三次对照、旧版本结果和 bdz1 04/05/06 结果见算法仓库
`docs/coordination_extended_validation_zh.md`。

```bash
python3 scripts/generate_sxbdz1_extended_benchmarks.py
python3 scripts/run_extended_coordination_experiments.py --repeats 3
```

该实验脚本记录失败并继续后续场景；可用 `--scenario` 选择指定配置。成功场景的完整动画位于
各自 `output/sxbdz1/06_...` 至 `09_...`，与 01 的输出分开保存。
生成器加 `--plot` 可重建 PNG/PDF 覆盖对比图，需要额外安装 matplotlib。
