#!/usr/bin/env python3
"""Audit single-map trajectories, allocations, clearance and all GIF frames.

Run after capability_mission_planner_cli. Dependencies: numpy, opencv-python,
PyYAML and Pillow. Writes validation.json and validation.txt beside plan.json.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', nargs='?', type=Path,
                        default=ROOT / 'configs/sxbdz1/01_six_robots_80_sites_100_tasks.yaml')
    parser.add_argument('--run-log', type=Path)
    parser.add_argument('--allow-zero-waits', action='store_true',
                        help='allow naturally conflict-free configurations (e.g. bdz1/06)')
    parser.add_argument('--skip-gif', action='store_true', help='audit complete JSON trajectories without decoding GIF')
    args = parser.parse_args()
    config_path = args.config.resolve()
    config = yaml.safe_load(config_path.read_text())
    output = (config_path.parent / config['output_directory']).resolve()
    plan = json.loads((output / 'plan.json').read_text())
    require(len(plan['maps']) == 1, 'this audit requires one map')
    map_info = plan['maps'][0]
    w, h, resolution = map_info['width'], map_info['height'], map_info['resolution']
    metadata = yaml.safe_load((config_path.parent / config['map']['directory'] / (map_info['id'] + '.yaml')).read_text())
    gray = cv2.imread(str(config_path.parent / config['map']['directory'] / metadata['image']), cv2.IMREAD_GRAYSCALE)
    require(gray.shape == (h, w), 'exported dimensions differ from the occupancy map')
    occupancy = (255 - gray.astype(float)) / 255
    if metadata.get('negate', 0):
        occupancy = 1 - occupancy
    free = (occupancy <= metadata['free_thresh']).astype(np.uint8)
    clearance = cv2.distanceTransform(free, cv2.DIST_L2, 3) * resolution
    ox, oy, yaw = metadata['origin']
    require(yaw == 0, 'this audit currently expects zero map origin yaw')

    def cell(position):
        if 'grid' in position:
            return tuple(position['grid'])
        return tuple(int(np.floor((v - o) / resolution + 1e-8))
                     for v, o in zip(position['local_xy'], (ox, oy)))

    robots = {r['id']: r for r in config['robots']}
    tasks = {t['id']: t for t in config['tasks']}
    expected = config['test_expectations']
    require(len(tasks) == expected['tasks'], 'unexpected atomic task count')
    require(len(plan['routes']) == len(robots) == expected['robots'], 'unexpected robot count')
    require({r['robot'] for r in plan['routes']} == set(robots), 'robot route IDs differ')
    assignment = Counter()
    merged = idle = total_wait_ticks = 0
    minimum_clearance = float('inf')
    horizon = max(r['timed_schedule'][-1]['tick'] for r in plan['routes']) + 1
    schedules = []
    per_robot = []
    resource_names = []
    for route in plan['routes']:
        robot = robots[route['robot']]
        schedule = route['timed_schedule']
        require([s['tick'] for s in schedule] == list(range(len(schedule))), 'schedule has missing/duplicate ticks')
        require(all(s['map_id'] == map_info['id'] for s in schedule), 'unknown map in schedule')
        coords = np.array([s['grid'] for s in schedule], dtype=int)
        require(tuple(coords[0]) == cell(robot['start']), 'robot start changed')
        require(tuple(coords[-1]) == cell(robot['return_home']), 'robot did not return home')
        require(np.all((coords[:, 0] >= 0) & (coords[:, 0] < w) &
                       (coords[:, 1] >= 0) & (coords[:, 1] < h)), 'trajectory leaves map bounds')
        values = clearance[h - 1 - coords[:, 1], coords[:, 0]]
        required_clearance = robot.get('clearance_radius_m', 0) + robot.get('safety_margin_m', 0)
        require(float(values.min()) + 1e-6 >= required_clearance, 'trajectory violates obstacle clearance')
        minimum_clearance = min(minimum_clearance, float(values.min()))
        # Rasterize each actual spatial segment, including conservative coarse paths.
        moved = coords[np.r_[True, np.any(np.diff(coords, axis=0) != 0, axis=1)]]
        segment_mask = np.zeros((h, w), dtype=np.uint8)
        for before, after in zip(moved[:-1], moved[1:]):
            cv2.line(segment_mask, (int(before[0]), h - 1 - int(before[1])),
                     (int(after[0]), h - 1 - int(after[1])), 1, 1)
        if segment_mask.any():
            along = clearance[segment_mask > 0]
            require(float(along.min()) + 1e-6 >= required_clearance, 'route segment violates obstacle clearance')
            minimum_clearance = min(minimum_clearance, float(along.min()))
        checkpoints = {p['task']: p for p in route['navigation_checkpoints'] if p['type'] == 'task'}
        service_seconds = 0
        assigned_count = 0
        for stop in route['stops']:
            require(bool(stop['tasks']), 'empty task stop')
            merged += len(stop['tasks']) > 1
            stop_service = 0
            for task_id in stop['tasks']:
                require(task_id in tasks, 'unknown assigned task')
                assignment[task_id] += 1
                task = tasks[task_id]
                require(set(task['requirements']) <= set(robot['capabilities']), 'capability mismatch')
                require(cell(stop) == cell(task['location']), 'assigned task position changed')
                require(task_id in checkpoints, 'task navigation checkpoint missing')
                checkpoint = checkpoints[task_id]
                arrival, departure = checkpoint['arrival_tick'], checkpoint['departure_tick']
                require(0 <= arrival <= departure < len(schedule), 'invalid task timing')
                require(cell(checkpoint) == cell(task['location']), 'task checkpoint position changed')
                require(tuple(coords[arrival]) == cell(task['location']), 'task arrival trajectory inconsistent')
                require(np.all(coords[arrival:departure + 1] == coords[arrival]), 'robot leaves during task service')
                require((departure - arrival) * plan['time_step_seconds'] + 1e-6 >= task['service_seconds'],
                        'task service interval is too short')
                service_seconds += task['service_seconds']
                stop_service += task['service_seconds']
                assigned_count += 1
            require(abs(stop['service_ticks'] * plan['time_step_seconds'] - stop_service) < 1e-6,
                    'merged stop service duration inconsistent')
        require(abs(route['service_ticks'] * plan['time_step_seconds'] - service_seconds) < 1e-6,
                'robot service time differs from assigned tasks')
        idle += assigned_count == 0
        route_frames = np.array([s['route_frame'] for s in schedule])
        waits = int((np.diff(route_frames) == 0).sum())
        event_ticks = sum(e['end_tick'] - e['start_tick'] for e in route['traffic_events'] if e['type'] == 'wait')
        require(waits == event_ticks, 'traffic events differ from actual inserted waits')
        for event in route['traffic_events']:
            require(0 <= event['start_tick'] < event['end_tick'] < len(schedule), 'invalid wait event')
            require(np.all(route_frames[event['start_tick']:event['end_tick'] + 1] ==
                           route_frames[event['start_tick']]), 'wait event contains route progress')
        total_wait_ticks += waits
        padded = np.vstack((coords, np.repeat(coords[-1:], horizon - len(coords), axis=0)))
        schedules.append(padded.astype(float) * resolution)
        resources = [s['resource'] for s in schedule]
        resources.extend([resources[-1]] * (horizon - len(resources)))
        resource_names.append(resources)
        per_robot.append({'robot': robot['id'], 'tasks': assigned_count, 'stops': len(route['stops']),
                          'finish_seconds': round((len(schedule) - 1) * plan['time_step_seconds'], 3),
                          'wait_seconds': round(waits * plan['time_step_seconds'], 3),
                          'wait_events': len(route['traffic_events']),
                          'minimum_clearance_m': round(float(values.min()), 6)})
    require(assignment == Counter({k: 1 for k in tasks}), 'tasks not assigned exactly once')
    require(idle == expected['idle_robots'], 'unexpected idle robots')
    require(merged >= expected['minimum_merged_stops'], 'too few merged stops')
    require(total_wait_ticks == plan['coordination']['total_wait_ticks'], 'coordination wait total differs')
    if not args.allow_zero_waits:
        require(total_wait_ticks > 0, 'benchmark must exercise actual coordination waiting')
    minimum_separation = float('inf')
    pairs = 0
    for a in range(len(schedules)):
        for b in range(a + 1, len(schedules)):
            ra, rb = robots[per_robot[a]['robot']], robots[per_robot[b]['robot']]
            threshold = sum(r.get('footprint_radius_m', 0) + r.get('safety_margin_m', 0) for r in (ra, rb))
            delta = schedules[a] - schedules[b]
            vertex = np.linalg.norm(delta, axis=1)
            require(float(vertex.min()) > threshold + 1e-9, 'vertex/footprint conflict')
            velocity = np.diff(delta, axis=0)
            denom = np.einsum('ij,ij->i', velocity, velocity)
            dot = np.einsum('ij,ij->i', delta[:-1], velocity)
            u = np.clip(np.divide(-dot, denom, out=np.zeros_like(dot), where=denom > 0), 0, 1)
            swept = np.linalg.norm(delta[:-1] + velocity * u[:, None], axis=1)
            require(float(swept.min()) > threshold + 1e-9, 'swept footprint conflict')
            minimum_separation = min(minimum_separation, float(vertex.min()), float(swept.min()))
            require(not any(x and x == y for x, y in zip(resource_names[a], resource_names[b])),
                    'same edge/transition resource occupied simultaneously')
            pairs += 1
    # Baseline: remove only coordination waits, retaining route travel and service.
    # This establishes that waiting in the result responds to actual collisions.
    baseline = []
    for route in plan['routes']:
        schedule = route['timed_schedule']
        states = [s for i, s in enumerate(schedule) if i == 0 or s['route_frame'] != schedule[i - 1]['route_frame']]
        baseline.append(np.array([s['grid'] for s in states], dtype=float) * resolution)
    baseline_unsafe_ticks = 0
    for a in range(len(baseline)):
        for b in range(a + 1, len(baseline)):
            length = max(len(baseline[a]), len(baseline[b]))
            aa = np.vstack((baseline[a], np.repeat(baseline[a][-1:], length - len(baseline[a]), axis=0)))
            bb = np.vstack((baseline[b], np.repeat(baseline[b][-1:], length - len(baseline[b]), axis=0)))
            ra, rb = robots[per_robot[a]['robot']], robots[per_robot[b]['robot']]
            threshold = sum(r.get('footprint_radius_m', 0) + r.get('safety_margin_m', 0) for r in (ra, rb))
            baseline_unsafe_ticks += int((np.linalg.norm(aa - bb, axis=1) <= threshold + 1e-9).sum())
    if not args.allow_zero_waits:
        require(baseline_unsafe_ticks > 0, 'uncoordinated baseline should contain conflicts')
    gif_path = output / 'routes_overall.gif'
    duration = 0
    amber_frames = 0
    gif_result = None
    if not args.skip_gif:
        gif = Image.open(gif_path)
        require(gif.size[0] == config['export']['animation_width'], 'GIF width incorrect')
        require(2 <= gif.n_frames <= config['export']['animation_max_frames'], 'GIF frame limit violated')
        require(gif.info.get('loop') == 0, 'GIF must loop')
        for frame_index in range(gif.n_frames):
            gif.seek(frame_index)
            gif.load()
            duration += gif.info['duration']
            rgb = np.array(gif.convert('RGB'))
            # Exclude header/sidebar; look for actual wait halos over the map.
            map_pixels = rgb[125:, :gif.size[0] - 300]
            amber_frames += bool(np.any(np.all(map_pixels == [255, 153, 0], axis=2)))
        require(gif.info['duration'] == 1000, 'GIF final-frame pause missing')
        if total_wait_ticks:
            require(amber_frames > 0, 'GIF does not show coordination wait halos')
        gif_result = {'pixels': gif.size, 'frames': gif.n_frames, 'seconds': duration / 1000,
                      'bytes': gif_path.stat().st_size, 'frames_with_wait_halo': amber_frames}
    times = {}
    if args.run_log:
        for line in args.run_log.read_text().splitlines():
            if ':' in line:
                key, value = line.strip().split(':', 1)
                if key.startswith('timing_') or key == 'peak_rss_mb':
                    times[key] = float(value)
    result = {
        'passed': True, 'map_pixels': [w, h], 'map_resolution_m': resolution,
        'robots': len(robots), 'unique_sites': len({cell(t['location']) for t in tasks.values()}),
        'tasks': len(tasks), 'categories': len({t['category'] for t in tasks.values()}),
        'requirement_sizes': dict(sorted(Counter(len(t['requirements']) for t in tasks.values()).items())),
        'high_priority_tasks': sum(t.get('high_priority', False) for t in tasks.values()),
        'merged_stops': merged, 'idle_robots': idle, 'coordination': plan['coordination'],
        'checked_ticks': horizon, 'checked_robot_pairs': pairs,
        'baseline_unsafe_pair_ticks': baseline_unsafe_ticks,
        'coordinated_unsafe_pair_ticks': 0,
        'minimum_swept_separation_m': round(minimum_separation, 6),
        'minimum_obstacle_clearance_m': round(minimum_clearance, 6),
        'makespan_seconds': round((horizon - 1) * plan['time_step_seconds'], 3),
        'total_wait_seconds': round(total_wait_ticks * plan['time_step_seconds'], 3),
        'per_robot': per_robot,
        'gif': gif_result,
        'runtime': times,
    }
    (output / 'validation.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    rows = [f"{r['robot']}: tasks={r['tasks']} stops={r['stops']} finish={r['finish_seconds']}s "
            f"wait={r['wait_seconds']}s events={r['wait_events']}" for r in per_robot]
    (output / 'validation.txt').write_text(
        f"PASS: {len(robots)} robots / {result['unique_sites']} sites / {len(tasks)} tasks / {result['categories']} categories\n"
        f"method={plan['coordination']['method']} merged_stops={merged} idle={idle}\n"
        f"baseline unsafe pair-ticks={baseline_unsafe_ticks}; coordinated unsafe pair-ticks=0\n"
        f"minimum swept separation={minimum_separation:.6f}m; obstacle clearance={minimum_clearance:.6f}m\n"
        f"makespan={result['makespan_seconds']}s; total waiting={result['total_wait_seconds']}s\n" +
        '\n'.join(rows) + '\n' + json.dumps(result['gif']) + '\n')
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
