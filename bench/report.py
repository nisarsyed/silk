#!/usr/bin/env python3
"""Bounded standard-library validation, repeat runs and matched comparisons."""
import argparse
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import sys

SCENES = ('pyramid', 'rain', 'piles', 'chains', 'churn', 'table', 'inverted')
WORK = ('tree_node_visits', 'pair_candidates', 'pair_probes', 'proxy_creates',
        'proxy_destroys', 'proxy_moves', 'contact_drops', 'graph_body_visits', 'graph_constraint_visits', 'graph_parent_probes')
QUALITY = ('penetration_max', 'cached_penetration_max', 'translation_drift_max',
           'rotation_drift_max', 'linear_speed_max', 'angular_speed_max',
           'joint_error_max', 'support_force_mean', 'supported_weight')
MEMORY = ('island', 'world_state', 'body', 'broadphase', 'contact', 'pair', 'contact_solver', 'joint', 'padding', 'arena', 'world')
SETTINGS = ('fixture_version', 'friction', 'restitution', 'seed', 'warmup_steps', 'measured_steps', 'dt_seconds', 'substeps',
            'body_capacity', 'contact_capacity', 'joint_capacity', 'gravity',
            'linear_drag', 'angular_drag', 'sleep_enabled', 'linear_speed_max',
            'contact_hertz', 'contact_damping_ratio', 'contact_push_velocity_max',
            'restitution_threshold', 'joint_hertz', 'joint_damping_ratio')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fields(value, names, path):
    require(type(value) is dict and set(value) == set(names), f'{path}: incorrect fields')


def number(value, low, high, path, integer=False):
    require(type(value) in ((int,) if integer else (int, float)), f'{path}: wrong numeric type')
    require(math.isfinite(value) and low <= value <= high, f'{path}: out of range/non-finite')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f'duplicate key: {key}')
        result[key] = value
    return result


def decode(text):
    require(len(text.encode('utf-8')) <= 1_048_576, 'report exceeds 1 MiB')
    return json.loads(text, object_pairs_hook=unique_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f'non-finite {value}')))


def load(path):
    with Path(path).open('rb') as stream:
        data = stream.read(1_048_577)
    require(len(data) <= 1_048_576, 'report exceeds 1 MiB')
    return decode(data.decode('utf-8'))


def validate(report):
    fields(report, ('schema_version', 'metadata', 'results'), 'report')
    require(type(report['schema_version']) is int and report['schema_version'] == 3, 'unsupported schema')
    meta = report['metadata']
    fields(meta, ('compiler', 'compiler_version', 'build', 'flags', 'warnings', 'host',
                  'host_version', 'processor', 'revision'), 'metadata')
    for key, value in meta.items():
        require(type(value) is str and 0 < len(value) <= 4096, f'metadata.{key}: invalid string')
    results = report['results']
    require(type(results) is list and 1 <= len(results) <= len(SCENES), 'invalid results count')
    seen = set()
    for row in results:
        fields(row, ('scene', 'settings', 'timing_ms', 'semantic_digest',
                     'drops', 'counts', 'step', 'cumulative_work', 'memory_bytes', 'quality'), 'result')
        scene = row['scene']
        require(type(scene) is str and scene in SCENES and scene not in seen, 'invalid/duplicate scene')
        seen.add(scene)
        settings = row['settings']
        fields(settings, SETTINGS, 'settings')
        for key, high in (('fixture_version', 1), ('seed', 2**32-1), ('warmup_steps', 10000), ('measured_steps', 10000),
                          ('substeps', 8), ('body_capacity', 65536), ('contact_capacity', 262144),
                          ('joint_capacity', 65536)):
            number(settings[key], 0 if key in ('seed', 'warmup_steps', 'joint_capacity') else 1,
                   high, f'settings.{key}', True)
        require(type(settings['sleep_enabled']) is bool and not settings['sleep_enabled'], 'wave 1 is awake')
        require(type(settings['gravity']) is list and len(settings['gravity']) == 2, 'gravity shape')
        for value in settings['gravity']:
            number(value, -1e6, 1e6, 'gravity')
        for key in SETTINGS:
            if key not in ('fixture_version', 'seed', 'warmup_steps', 'measured_steps', 'substeps', 'body_capacity',
                           'contact_capacity', 'joint_capacity', 'sleep_enabled', 'gravity'):
                number(settings[key], 0 if key.endswith('drag') or key in ('friction', 'restitution') else 1e-30, 1e30, key)
        number(settings['restitution'], 0, 1, 'restitution')
        timing = row['timing_ms']
        fields(timing, ('average', 'median', 'p95', 'max', 'mutation_average'), 'timing')
        for key, value in timing.items():
            number(value, 0, 1e12, f'timing.{key}')
        require(timing['median'] <= timing['p95'] <= timing['max'] and timing['average'] <= timing['max'], 'timing order')
        require(type(row['semantic_digest']) is str and re.fullmatch(r'[0-9a-f]{16}', row['semantic_digest']), 'invalid semantic_digest')
        number(row['drops'], 0, 0, 'unexpected contact drops', True)
        counts = row['counts']
        fields(counts, ('bodies', 'contacts', 'joints', 'pairs', 'pair_capacity',
                        'body_high', 'contact_high', 'joint_high'), 'counts')
        for key, value in counts.items():
            number(value, 0, 524288, f'counts.{key}', True)
        for plural, singular in (('bodies', 'body'), ('contacts', 'contact'), ('joints', 'joint')):
            require(counts[plural] <= counts[singular + '_high'] <= settings[singular + '_capacity'], 'pool count invariant')
        require(counts['pairs'] == counts['contacts'] and counts['pairs'] * 2 <= counts['pair_capacity'], 'pair invariant')
        capacity = counts['pair_capacity']
        require(capacity > 0 and capacity & (capacity - 1) == 0, 'pair capacity must be power of two')
        step = row['step']
        fields(step, ('dynamic_bodies', 'kinematic_bodies', 'contact_constraints', 'joint_constraints', 'substeps', 'islands', 'island_bodies_max', 'work'), 'step')
        for key in ('dynamic_bodies', 'kinematic_bodies', 'contact_constraints', 'joint_constraints', 'substeps', 'islands', 'island_bodies_max'):
            number(step[key], 0, 262144, f'step.{key}', True)
        require(step['substeps'] == settings['substeps'], 'step substeps mismatch')
        require(step['dynamic_bodies'] + step['kinematic_bodies'] <= counts['bodies'], 'active count invariant')
        require(step['contact_constraints'] <= counts['contacts'] and step['joint_constraints'] <= counts['joints'], 'constraint count invariant')
        for work in (step['work'], row['cumulative_work']):
            fields(work, WORK, 'work')
            for key in WORK:
                number(work[key], 0, 2**64 - 1, key, True)
            require(work['contact_drops'] == 0, 'unexpected work drops')
        require(all(step['work'][k] <= row['cumulative_work'][k] for k in WORK), 'work interval invariant')
        memory = row['memory_bytes']
        fields(memory, MEMORY, 'memory')
        for key, value in memory.items():
            number(value, 0, 2**64 - 1, f'memory.{key}', True)
        require(sum(memory[k] for k in MEMORY[:-2]) == memory['arena'], 'memory sum mismatch')
        require(memory['world'] > 0 and memory['arena'] > 0, 'empty memory budget')
        quality = row['quality']
        fields(quality, ('window_steps',) + QUALITY, 'quality')
        number(quality['window_steps'], 1, 60, 'window_steps', True)
        require(quality['window_steps'] == min(60, settings['measured_steps']), 'quality window mismatch')
        for key in QUALITY:
            number(quality[key], -1e12 if key == 'support_force_mean' else 0, 1e12, key)
    return report


def quality_check(report):
    limits = load(Path(__file__).with_name('quality_limits.json'))
    for row in report['results']:
        settings = row['settings']
        require(settings['warmup_steps'] == 120 and settings['measured_steps'] == 600, 'quality gate requires 120/600 profile')
        bound = limits[row['scene']]
        for metric, high in bound['maximum'].items():
            require(row['quality'][metric] <= high, f"{row['scene']}: {metric} exceeds {high}")
        if 'support_ratio' in bound:
            q = row['quality']
            require(q['supported_weight'] > 0, 'support weight is zero')
            ratio = q['support_force_mean'] / q['supported_weight']
            require(bound['support_ratio'][0] <= ratio <= bound['support_ratio'][1], f"{row['scene']}: load transfer ratio {ratio}")


def deterministic(report):
    return [{k: v for k, v in row.items() if k != 'timing_ms'} for row in report['results']]


def compare(a, b, cross_build=False):
    validate(a); validate(b)
    require([r['scene'] for r in a['results']] == [r['scene'] for r in b['results']], 'scene selection mismatch')
    require([r['settings'] for r in a['results']] == [r['settings'] for r in b['results']], 'workload/settings mismatch')
    if not cross_build:
        require(deterministic(a) == deterministic(b), 'same-binary replay mismatch')
    else:
        quality_check(a); quality_check(b)
    return [{
        'scene': x['scene'],
        'average_ms_before': x['timing_ms']['average'],
        'average_ms_after': y['timing_ms']['average'],
        'arena_delta_bytes': y['memory_bytes']['arena'] - x['memory_bytes']['arena'],
        'world_delta_bytes': y['memory_bytes']['world'] - x['memory_bytes']['world'],
        'digest_equal': x['semantic_digest'] == y['semantic_digest'],
        'work_delta': {k: y['cumulative_work'][k] - x['cumulative_work'][k] for k in WORK},
        'quality_delta': {k: y['quality'][k] - x['quality'][k] for k in QUALITY},
    } for x, y in zip(a['results'], b['results'])]


def bounded(low, high):
    def parse(text):
        if not re.fullmatch(r'[0-9]{1,5}', text) or not low <= int(text) <= high:
            raise argparse.ArgumentTypeError(f'expected {low}..{high}')
        return int(text)
    return parse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    run = commands.add_parser('run')
    run.add_argument('executable', type=Path)
    run.add_argument('--output', type=Path, required=True)
    run.add_argument('--repeat', type=bounded(1, 20), default=5)
    run.add_argument('--scene', choices=('all',) + SCENES, default='all')
    run.add_argument('--warmup', type=bounded(0, 10000), default=120)
    run.add_argument('--steps', type=bounded(1, 10000), default=600)
    run.add_argument('--quality', action='store_true')
    check = commands.add_parser('validate')
    check.add_argument('report', type=Path)
    check.add_argument('--quality', action='store_true')
    diff = commands.add_parser('compare')
    diff.add_argument('before', type=Path)
    diff.add_argument('after', type=Path)
    diff.add_argument('--cross-build', action='store_true')
    args = parser.parse_args()
    try:
        if args.command == 'validate':
            report = validate(load(args.report))
            if args.quality:
                quality_check(report)
            print('valid')
        elif args.command == 'compare':
            print(json.dumps(compare(load(args.before), load(args.after), args.cross_build), indent=2))
        else:
            args.output.mkdir(parents=True, exist_ok=True)
            command = [str(args.executable.resolve()), '--scene', args.scene, '--warmup', str(args.warmup),
                       '--steps', str(args.steps)]
            reports = []
            for i in range(args.repeat):
                result = subprocess.run(command, text=True, capture_output=True, timeout=600, check=False)
                (args.output / f'run-{i+1}.json').write_text(result.stdout)
                (args.output / f'run-{i+1}.stderr.txt').write_text(result.stderr)
                require(result.returncode == 0, f'benchmark exited {result.returncode}: {result.stderr}')
                report = validate(decode(result.stdout))
                expected = SCENES if args.scene == 'all' else (args.scene,)
                require(tuple(r['scene'] for r in report['results']) == expected, 'missing requested scene')
                if args.quality:
                    quality_check(report)
                if reports:
                    compare(reports[0], report)
                reports.append(report)
            summary = {'command': command, 'repetitions': args.repeat, 'metadata': reports[0]['metadata'], 'scenes': []}
            for index, row in enumerate(reports[0]['results']):
                samples = [r['results'][index]['timing_ms']['average'] for r in reports]
                summary['scenes'].append({'scene': row['scene'], 'average_ms_runs': samples,
                    'median_average_ms': statistics.median(samples), 'minimum_average_ms': min(samples),
                    'maximum_average_ms': max(samples)})
            (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
            print(json.dumps(summary, indent=2))
        return 0
    except (ValueError, TypeError, KeyError, OverflowError, RecursionError, OSError, subprocess.TimeoutExpired) as error:
        print(f'benchmark report: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
