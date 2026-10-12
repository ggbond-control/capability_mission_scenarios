#!/usr/bin/env python3
"""Generate a reproducible, clearance-aware 80-site/100-task sxbdz1 benchmark.

Dependencies: numpy, opencv-python, PyYAML. Run from any working directory.
The selected points belong to one conservative factor-4 connected component.
"""
from pathlib import Path
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CAPABILITIES = [
    ('fire-camera-gas', ['fire', 'camera', 'gas']),
    ('camera-thermal-delivery', ['camera', 'thermal', 'delivery']),
    ('fire-thermal-delivery', ['fire', 'thermal', 'delivery']),
    ('fire-delivery-gas', ['fire', 'delivery', 'gas']),
    ('camera-thermal-gas', ['camera', 'thermal', 'gas']),
    ('camera-delivery-gas', ['camera', 'delivery', 'gas']),
]
TYPES = [
    ('fire_suppression', ['fire']), ('visual_inspection', ['camera']),
    ('thermal_inspection', ['thermal']), ('material_delivery', ['delivery']),
    ('gas_detection', ['gas']), ('fire_documentation', ['fire', 'camera']),
    ('heat_hazard_assessment', ['fire', 'thermal']),
    ('emergency_supply', ['fire', 'delivery']), ('combustion_gas_check', ['fire', 'gas']),
    ('thermal_documentation', ['camera', 'thermal']),
    ('delivery_verification', ['camera', 'delivery']),
    ('gas_documentation', ['camera', 'gas']),
    ('cold_chain_delivery', ['thermal', 'delivery']),
    ('thermal_gas_survey', ['thermal', 'gas']),
    ('gas_equipment_delivery', ['delivery', 'gas']),
] + [('integrated_' + name.replace('-', '_'), caps) for name, caps in CAPABILITIES]


def main():
    map_dir = ROOT / 'maps/sxbdz1'
    metadata = yaml.safe_load((map_dir / 'map_000.yaml').read_text())
    image = cv2.imread(str(map_dir / metadata['image']), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError('cannot read sxbdz1 occupancy image')
    occupancy = (255 - image.astype(float)) / 255
    if metadata.get('negate', 0):
        occupancy = 1 - occupancy
    free = (occupancy <= metadata['free_thresh']).astype(np.uint8)
    clearance = cv2.distanceTransform(free, cv2.DIST_L2, 3) * metadata['resolution']
    factor = 4
    h, w = image.shape
    # Grid origin is at bottom left, so block in grid coordinates.
    safe = np.flipud((free > 0) & (clearance >= 0.40))
    coarse = safe[:h // factor * factor, :w // factor * factor].reshape(
        h // factor, factor, w // factor, factor).all(axis=(1, 3)).astype(np.uint8)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(coarse, connectivity=4)
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.where(labels == component)
    # Deterministic farthest-point sampling distributes sites over the whole map.
    candidates = np.column_stack((xs * factor + factor // 2, ys * factor + factor // 2))
    candidates = candidates[(candidates[:, 0] % 12 == 2) & (candidates[:, 1] % 12 == 2)]
    if len(candidates) < 86:
        raise RuntimeError('not enough connected clearance-safe candidate points')
    selected = []
    distances = np.full(len(candidates), np.inf)
    choice = int(np.argmin(candidates[:, 0] + candidates[:, 1]))
    for _ in range(86):
        point = candidates[choice]
        selected.append(point.tolist())
        distances = np.minimum(distances, ((candidates - point) ** 2).sum(axis=1))
        choice = int(np.argmax(distances))
    starts, sites = selected[:6], selected[6:]
    resolution = metadata['resolution']
    ox, oy, yaw = metadata['origin']
    if yaw != 0:
        raise RuntimeError('generator requires zero origin yaw')

    def position(cell):
        return {'map_id': 'map_000', 'local_xy': [round(ox + (cell[0] + .5) * resolution, 3),
                                                round(oy + (cell[1] + .5) * resolution, 3)]}

    robots = []
    for i, (name, caps) in enumerate(CAPABILITIES):
        robots.append({'id': name, 'start': position(starts[i]), 'capabilities': caps,
                       'return_home': position(starts[i]), 'clearance_radius_m': .25,
                       'footprint_radius_m': .25, 'safety_margin_m': .05,
                       'nominal_speed_mps': [.7, .8, .9, .7, .8, .9][i]})
    tasks = []
    for i, site in enumerate(sites):
        category, requirements = TYPES[i % len(TYPES)]
        task = {'id': f't{i + 1:03d}-{category}', 'location': position(site),
                'requirements': requirements, 'category': category,
                'service_seconds': 2 + (i * 7) % 11}
        if i % 7 == 0 or 'fire' in requirements:
            task['high_priority'] = True
        tasks.append(task)
    # Twenty extra atomic tasks share sites and the same exclusive triple capability.
    # Those pairs must go to the same robot and merge into one physical stop.
    exclusive = [i for i in range(80) if i % len(TYPES) >= 15][:20]
    for j, i in enumerate(exclusive):
        source = tasks[i]
        tasks.append({'id': f't{81 + j:03d}-followup', 'location': source['location'],
                      'requirements': source['requirements'], 'category': source['category'],
                      'service_seconds': 3 + j % 6})
    assert len(tasks) == 100
    config = {
        'test_expectations': {'robots': 6, 'tasks': 100, 'idle_robots': 0, 'minimum_merged_stops': 20},
        'map': {'directory': '../../maps/sxbdz1', 'allow_unknown': False,
                'inflation_radius_m': 0., 'inscribed_radius_m': .25, 'cost_scaling_factor': 5.},
        'output_directory': '../../output/sxbdz1/01_six_robots_80_sites_100_tasks',
        'planner': {'coordinate_conflicts': True,
                    'objective': {'maximum_load_weight': 1.5, 'total_load_weight': .1},
                    'traversal': {'time_step_seconds': .2, 'nominal_speed_mps': .8,
                                  'obstacle_cost_weight': 1., 'allow_diagonal': True,
                                  'downsample_costmap': True, 'coarse_search_factor': factor,
                                  'coordination_max_high_level_nodes': 512}},
        'export': {'path_thickness': 3, 'draw_grid': False, 'animation': True,
                   'animation_fps': 10, 'animation_speedup': 30.,
                   'animation_max_frames': 600, 'animation_width': 1600},
        'robots': robots, 'tasks': tasks,
    }
    target = ROOT / 'configs/sxbdz1/01_six_robots_80_sites_100_tasks.yaml'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text('# Generated by scripts/generate_sxbdz1_benchmark.py; deterministic clearance-aware sites.\n' +
                      yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
    print(f'{target}: 6 robots, 80 sites, 100 tasks, {len(TYPES)} categories')


if __name__ == '__main__':
    main()
