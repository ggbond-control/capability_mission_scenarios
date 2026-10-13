#!/usr/bin/env python3
"""Generate dense sxbdz1 cases, with separately deployed conservative service zones.

Does not overwrite the original 01 case or change the occupancy map. Requires
numpy, OpenCV and PyYAML. Zones use the original factor-4, >=0.4 m clearance rule.
"""
from pathlib import Path
import argparse
import copy
import json

import cv2
import numpy as np
import yaml

from generate_sxbdz1_benchmark import CAPABILITIES, TYPES

ROOT = Path(__file__).resolve().parents[1]
CASES = [
    ("02_six_robots_160_sites_220_tasks", [6], [160], 220, False),
    ("03_twelve_robots_240_sites_360_tasks", [12], [240], 360, False),
    ("04_twelve_robots_240_sites_360_tasks_two_zones", [6, 6], [170, 70], 360, False),
    ("05_twenty_robots_320_sites_480_tasks_ten_zones",
     [6, 6] + [1] * 8, [170, 70, 15, 12, 12, 10, 10, 8, 7, 6], 480, False),
    ("06_six_robots_160_sites_220_tasks_dispersed", [6], [160], 220, True),
    ("07_twelve_robots_240_sites_360_tasks_dispersed", [12], [240], 360, True),
    ("08_twelve_robots_240_sites_360_tasks_two_zones_dispersed", [6, 6], [170, 70], 360, True),
    ("09_twenty_robots_320_sites_480_tasks_ten_zones_dispersed",
     [6, 6] + [1] * 8, [170, 70, 15, 12, 12, 10, 10, 8, 7, 6], 480, True),
]


def farthest_points(candidates, count):
    if len(candidates) < count:
        raise ValueError(f"need {count} separated candidates, only {len(candidates)} available")
    distances = np.full(len(candidates), np.inf)
    choice = int(np.argmin(candidates[:, 0] + candidates[:, 1]))
    selected = []
    for _ in range(count):
        point = candidates[choice]
        selected.append(point)
        distances = np.minimum(distances, np.square(candidates - point).sum(axis=1))
        choice = int(np.argmax(distances))
    return np.array(selected)


def coverage(labels, zone_labels, sites, resolution):
    ys, xs = np.where(np.isin(labels, zone_labels))
    points = np.column_stack((xs * 4 + 2, ys * 4 + 2))
    distances = []
    for chunk in np.array_split(points, 32):
        if len(chunk):
            distances.extend(np.sqrt(np.square(chunk[:, None] - sites).sum(axis=2).min(axis=1)) * resolution)
    bins = {(int(x) // 80, int(y) // 80) for x, y in points}
    site_bins = {(int(x) // 80, int(y) // 80) for x, y in sites}
    return {"zones": len(zone_labels), "conservative_coarse_cells_total": int((labels > 0).sum()),
            "deployed_zone_coarse_cells": len(points),
            "deployed_zone_area_fraction": len(points) / int((labels > 0).sum()),
            "zone_8m_bins": len(bins), "task_8m_bins": len(site_bins),
            "task_bin_coverage_fraction": len(bins & site_bins) / len(bins),
            "nearest_task_distance_p95_m": float(np.percentile(distances, 95)),
            "nearest_task_distance_max_m": float(max(distances)),
            "task_grid_bbox": [sites.min(axis=0).tolist(), sites.max(axis=0).tolist()]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plot', action='store_true', help='also export PNG/PDF coverage comparison (requires matplotlib)')
    args = parser.parse_args()
    directory = ROOT / "maps/sxbdz1"
    metadata = yaml.safe_load((directory / "map_000.yaml").read_text())
    image = cv2.imread(str(directory / metadata["image"]), cv2.IMREAD_GRAYSCALE)
    occupancy = (255 - image.astype(float)) / 255
    if metadata.get("negate", 0):
        occupancy = 1 - occupancy
    free = (occupancy <= metadata["free_thresh"]).astype(np.uint8)
    clearance = cv2.distanceTransform(free, cv2.DIST_L2, 3) * metadata["resolution"]
    safe = np.flipud((free > 0) & (clearance >= .4))
    height, width = safe.shape
    coarse = safe[:height // 4 * 4, :width // 4 * 4].reshape(
        height // 4, 4, width // 4, 4).all(axis=(1, 3)).astype(np.uint8)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(coarse, connectivity=4)
    planning_safe = np.flipud((free > 0) & (clearance >= .3))
    planning_coarse = planning_safe[:height // 4 * 4, :width // 4 * 4].reshape(
        height // 4, 4, width // 4, 4).all(axis=(1, 3)).astype(np.uint8)
    _, planning_labels = cv2.connectedComponents(planning_coarse, connectivity=4)
    zones = (np.argsort(-stats[1:, cv2.CC_STAT_AREA], kind="stable") + 1).tolist()
    resolution = metadata["resolution"]
    ox, oy, yaw = metadata["origin"]
    if yaw != 0:
        raise ValueError("this generator expects sxbdz1's zero origin yaw")

    def position(cell):
        return {"map_id": "map_000", "local_xy": [round(ox + (int(cell[0]) + .5) * resolution, 3),
                                                  round(oy + (int(cell[1]) + .5) * resolution, 3)]}

    output = ROOT / "output/sxbdz1/extended_benchmarks"
    output.mkdir(parents=True, exist_ok=True)
    manifest = []
    original_path = ROOT / "configs/sxbdz1/01_six_robots_80_sites_100_tasks.yaml"
    template = yaml.safe_load(original_path.read_text())
    original_sites = np.unique(np.array([[round((t["location"]["local_xy"][0] - ox) / resolution - .5),
                                         round((t["location"]["local_xy"][1] - oy) / resolution - .5)]
                                        for t in template["tasks"]]), axis=0)
    manifest.append({"scenario": original_path.stem, "robots": 6, "sites": 80, "tasks": 100,
                     "coverage": coverage(labels, zones[:1], original_sites, resolution)})
    for name, robot_counts, site_counts, task_count, dispersed in CASES:
        robots, tasks, all_sites, zone_details = [], [], [], []
        multisector = len(robot_counts) > 1
        for zone, (robot_count, site_count) in enumerate(zip(robot_counts, site_counts)):
            ys, xs = np.where(labels == zones[zone])
            candidates = np.column_stack((xs * 4 + 2, ys * 4 + 2))
            # Retain original 1.2 m lattice where possible; smaller zones use
            # the full 0.4 m lattice, then enforce actual pairwise start safety.
            sparse = candidates[(candidates[:, 0] % 12 == 2) & (candidates[:, 1] % 12 == 2)]
            if dispersed:
                starts = farthest_points(sparse if len(sparse) >= robot_count else candidates, robot_count)
                available = candidates[~np.any(np.all(candidates[:, None] == starts, axis=2), axis=1)]
                # Use every clearance-safe coarse cell: a 1.2 m lattice alone
                # can miss an entire thin corridor with a different alignment.
                sites = farthest_points(available, site_count)
            else:
                if len(sparse) >= robot_count + site_count:
                    candidates = sparse
                selected = farthest_points(candidates, robot_count + site_count)
                sites, starts = selected[:site_count], selected[site_count:]
            if robot_count > 1:
                distances = np.sqrt(np.square(starts[:, None] - starts).sum(axis=2)) * resolution
                np.fill_diagonal(distances, np.inf)
                if distances.min() <= .6:
                    raise ValueError("robot starts overlap")
            zone_cap = f"zone_{zone + 1:02d}"
            for robot_index in range(robot_count):
                basic_name, caps = CAPABILITIES[robot_index % len(CAPABILITIES)]
                if robot_count == 1:
                    basic_name, caps = "generalist", ["fire", "camera", "thermal", "delivery", "gas"]
                robot_id = f"{basic_name}-{robot_index + 1:02d}" if not multisector else f"{zone_cap}-{basic_name}-{robot_index + 1:02d}"
                robots.append({"id": robot_id, "start": position(starts[robot_index]),
                               "return_home": position(starts[robot_index]),
                               "capabilities": list(caps) + ([zone_cap] if multisector else []),
                               "clearance_radius_m": .25, "footprint_radius_m": .25,
                               "safety_margin_m": .05,
                               "nominal_speed_mps": [.7, .8, .9, .7, .8, .9][robot_index % 6]})
            for site_index, site in enumerate(sites):
                category, requirements = TYPES[site_index % len(TYPES)]
                tasks.append({"id": f"t{len(tasks) + 1:04d}-{category}", "location": position(site),
                              "requirements": list(requirements) + ([zone_cap] if multisector else []),
                              "category": category, "service_seconds": 2 + (site_index * 7 + zone * 3) % 19,
                              "high_priority": site_index % 7 == 0 or "fire" in requirements})
            all_sites.extend(sites.tolist())
            planning_components = np.unique(planning_labels[labels == zones[zone]]).tolist()
            if len(planning_components) != 1 or planning_components[0] == 0:
                raise ValueError("sampling zone is not contained in one planning-clearance component")
            zone_details.append({"zone": zone_cap, "component_label": zones[zone], "robots": robot_count,
                                 "planning_clearance_component": planning_components[0],
                                 "sites": site_count, "coarse_cells": int(stats[zones[zone], cv2.CC_STAT_AREA])})
        # Follow-up work preserves capability requirements at dispersed sites.
        sources = [task for task in tasks if len([c for c in task["requirements"] if not c.startswith("zone_")]) == 3]
        for index in range(task_count - len(tasks)):
            source = sources[index % len(sources)]
            followup = copy.deepcopy(source)
            followup.update(id=f"t{len(tasks) + 1:04d}-followup", service_seconds=3 + index * 5 % 11,
                            high_priority=index % 5 == 0)
            tasks.append(followup)
        config = copy.deepcopy(template)
        config.update(output_directory=f"../../output/sxbdz1/{name}", robots=robots, tasks=tasks,
                      test_expectations={"robots": len(robots), "tasks": task_count, "idle_robots": 0,
                                         "minimum_merged_stops": 0})
        config["export"].update(animation_speedup=60., animation_max_frames=300)
        target = ROOT / "configs/sxbdz1" / f"{name}.yaml"
        target.write_text("# Generated by scripts/generate_sxbdz1_extended_benchmarks.py\n" +
                          yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
        entry = {"scenario": name, "robots": len(robots), "sites": len(all_sites), "tasks": len(tasks),
                 "deployment": "dispersed" if dispersed else "interior",
                 "planning_clearance_components": len({z["planning_clearance_component"] for z in zone_details}),
                 "categories": len({t["category"] for t in tasks}), "zones": zone_details,
                 "coverage": coverage(labels, zones[:len(robot_counts)], np.array(all_sites), resolution)}
        manifest.append(entry)
        print(json.dumps(entry, ensure_ascii=False), flush=True)
    (output / "coverage.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        names = [manifest[index]['scenario'] for index in (0, 6, 8)]
        metrics = {case['scenario']: case['coverage'] for case in manifest}
        figure, axes = plt.subplots(1, 3, figsize=(16, 5.9), sharex=True, sharey=True, layout='constrained')
        for axis, name in zip(axes, names):
            config = yaml.safe_load((ROOT / 'configs/sxbdz1' / (name + '.yaml')).read_text())
            sites = np.unique([t['location']['local_xy'] for t in config['tasks']], axis=0)
            starts = np.array([robot['start']['local_xy'] for robot in config['robots']])
            axis.imshow(np.flipud(image), origin='lower', cmap='gray', vmin=0, vmax=255,
                        extent=[ox, ox + width * resolution, oy, oy + height * resolution])
            axis.scatter(sites[:, 0], sites[:, 1], s=6, c='#00c4ff', linewidths=0, label='Task sites')
            axis.scatter(starts[:, 0], starts[:, 1], s=28, c='#ffcf00', marker='+', linewidths=1, label='Robot starts')
            axis.set_title(f"{len(config['robots'])} robots / {len(sites)} sites / {len(config['tasks'])} tasks\n"
                           f"Zones cover {metrics[name]['deployed_zone_area_fraction']:.1%} of screened walkable area", fontsize=10)
            axis.set_xlabel('Map-local x (m)')
            axis.set_aspect('equal')
            axis.legend(loc='upper left', fontsize=8, framealpha=.9)
        axes[0].set_ylabel('Map-local y (m)')
        figure.savefig(output / 'map_coverage_comparison.png', dpi=180)
        figure.savefig(output / 'map_coverage_comparison.pdf')
        plt.close(figure)


if __name__ == "__main__":
    main()
