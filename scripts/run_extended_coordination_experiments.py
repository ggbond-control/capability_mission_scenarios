#!/usr/bin/env python3
"""Run bounded, reproducible coordination comparisons, retaining failed cases.

Default backends are current SIPP and SIPP plus bounded repair. Optional old
probes use the historical interface: PROBE CONFIG CSV. Each child gets the same
wall-clock / address-space limits. A failed backend is recorded, not retried.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import resource
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def frames(path):
    result, ticks = {}, {}
    with path.open() as file:
        for row in csv.DictReader(file):
            robot = row.pop('robot')
            tick = int(row.pop('tick'))
            if tick != ticks.get(robot, -1) + 1:
                raise ValueError('nonconsecutive schedule ticks')
            ticks[robot] = tick
            route = result.setdefault(robot, [])
            if not route or route[-1] != row:
                if int(row['frame']) != len(route):
                    raise ValueError('missing mandatory route/service frame')
                route.append(row)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path,
                        default=ROOT.parent / 'capability_mission_planner/build/coordination_benchmark')
    parser.add_argument('--scenario', action='append', help='config path relative to configs/, without .yaml')
    parser.add_argument('--output', type=Path, default=ROOT / 'output/extended_coordination_experiments')
    parser.add_argument('--legacy-probe', type=Path)
    parser.add_argument('--safe-tick-probe', type=Path)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=40)
    parser.add_argument('--memory-limit-mb', type=int, default=2048)
    args = parser.parse_args()
    if args.repeats < 1 or args.timeout <= 0 or args.memory_limit_mb < 1:
        parser.error('positive repeats, timeout and memory limit required')
    scenarios = args.scenario or [
        'bdz1/04_five_robot_types_50_points', 'bdz1/05_five_robot_types_60_points',
        'bdz1/06_five_robot_types_80_points',
    ] + [str(p.relative_to(ROOT / 'configs').with_suffix(''))
         for p in sorted((ROOT / 'configs/sxbdz1').glob('0[2-9]_*.yaml'))]
    backends = []
    if args.legacy_probe: backends.append(('original', args.legacy_probe.resolve()))
    if args.safe_tick_probe: backends.append(('safe_tick', args.safe_tick_probe.resolve()))
    backends += [('sipp', args.benchmark.resolve()), ('repair', args.benchmark.resolve())]
    args.output.mkdir(parents=True, exist_ok=True)
    memory_bytes = args.memory_limit_mb * 1024**2

    def child_limits():
        resource.setrlimit(resource.RLIMIT_AS, (memory_bytes, memory_bytes))

    results = {'limits': {'wall_seconds': args.timeout, 'address_space_mb': args.memory_limit_mb},
               'repeats': args.repeats, 'backends': {name: str(path) for name, path in backends}, 'cases': []}
    for scenario in scenarios:
        config = ROOT / 'configs' / (scenario + '.yaml')
        record = {'scenario': scenario, 'config_sha256': hashlib.sha256(config.read_bytes()).hexdigest(), 'backends': {}}
        sequences = {}
        for mode, executable in backends:
            csv_path = args.output / (scenario.replace('/', '__') + '__' + mode + '.csv')
            samples = []
            for repetition in range(args.repeats):
                command = [str(executable), str(config)]
                if mode in ('original', 'safe_tick'): command.append(str(csv_path))
                else:
                    if repetition == 0: command.append(f'--csv={csv_path}')
                    if mode == 'sipp': command.append('--no-repair')
                start = time.monotonic()
                try:
                    process = subprocess.run(command, capture_output=True, text=True,
                                             timeout=args.timeout, preexec_fn=child_limits)
                    sample = {'exit': process.returncode, 'metrics': json.loads(process.stdout) if process.stdout else {},
                              'error': process.stderr}
                except subprocess.TimeoutExpired:
                    sample = {'exit': 'timeout', 'metrics': {}, 'error': 'wall-clock limit reached'}
                sample['wall_seconds'] = time.monotonic() - start
                samples.append(sample)
                if sample['exit'] != 0: break
            data = {'samples': samples}
            if all(s['exit'] == 0 for s in samples):
                data['median_seconds'] = statistics.median(s['metrics']['seconds'] for s in samples)
                data['median_rss_mb'] = statistics.median(s['metrics']['rss_mb'] for s in samples)
                sequences[mode] = frames(csv_path)
            record['backends'][mode] = data
            first = samples[0]
            print(f'{scenario}/{mode}: exit={first["exit"]}, metrics={first["metrics"]}, '
                  f'error={first["error"].strip()}', flush=True)
        if 'sipp' in sequences and 'repair' in sequences:
            record['mandatory_frames_preserved'] = sequences['sipp'] == sequences['repair']
            if not record['mandatory_frames_preserved']: raise ValueError('repair changed mandatory frames')
            before = record['backends']['sipp']['samples'][0]['metrics']
            for sample in record['backends']['repair']['samples']:
                after = sample['metrics']
                if after['unsafe'] or any(after[k] > before[k] + 1e-6 for k in ('makespan', 'wait', 'total_finish')):
                    raise ValueError('unsafe or worse repaired schedule')
        for mode in ('original', 'safe_tick'):
            if mode in sequences and 'sipp' in sequences:
                record[mode + '_mandatory_frames_preserved'] = sequences[mode] == sequences['sipp']
                if not record[mode + '_mandatory_frames_preserved']: raise ValueError('static routes differ between backends')
        results['cases'].append(record)
        (args.output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')


if __name__ == '__main__':
    main()
