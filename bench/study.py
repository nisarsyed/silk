#!/usr/bin/env python3
"""Validate and repeat the fixed public solver-study fixtures."""
import argparse
import json
from pathlib import Path
import re
import statistics
import subprocess
import sys

from report import bounded, decode, fields, load, number, require

SCENES = ('bridge', 'collapse', 'zero-gravity', 'cycle', 'topple')
METRICS = ('drop_max', 'horizontal_motion_max', 'rotation_excursion_max',
           'penetration_max', 'translation_drift_max', 'rotation_drift_max',
           'linear_speed_max', 'angular_speed_max', 'cached_support_force_mean',
           'momentum_error_max')
META = ('revision', 'compiler', 'compiler_id', 'host', 'host_version', 'processor', 'flags')
SETTINGS = ('study_version', 'body_capacity', 'contact_capacity', 'seed',
            'quality_window', 'sleep_enabled', 'warmup', 'steps', 'substeps', 'dt')


def validate(value):
    fields(value, META + SETTINGS + ('results',), 'study')
    for key in META:
        require(isinstance(value[key], str) and 0 < len(value[key]) <= 4096, key)
    for key, expected in (('study_version', 1), ('body_capacity', 16),
                          ('contact_capacity', 128), ('seed', 0),
                          ('quality_window', 60), ('warmup', 120),
                          ('steps', 600), ('substeps', 4)):
        number(value[key], expected, expected, key, integer=True)
    require(type(value['sleep_enabled']) is bool, 'sleep_enabled')
    number(value['dt'], 0.01666666, 0.01666668, 'dt')
    require(isinstance(value['results'], list) and len(value['results']) == 5, 'results')
    for name, row in zip(SCENES, value['results']):
        fields(row, METRICS + ('scene', 'average_ms', 'digest', 'prepared_contacts',
                              'sleeping_dynamics', 'penetration_samples'), name)
        require(row['scene'] == name, 'scene order')
        require(isinstance(row['digest'], str) and re.fullmatch('[0-9a-f]{16}', row['digest']), 'digest')
        for key in METRICS + ('average_ms',):
            number(row[key], 0, 1e30, key)
        number(row['prepared_contacts'], 0, 128 * 600, 'prepared_contacts', integer=True)
        number(row['sleeping_dynamics'], 0, 16, 'sleeping_dynamics', integer=True)
        require(isinstance(row['penetration_samples'], list) and len(row['penetration_samples']) == 10,
                'penetration_samples')
        for sample in row['penetration_samples']:
            number(sample, 0, 1e30, 'penetration sample')
    return value


def deterministic(value):
    return {**{key: value[key] for key in SETTINGS},
            'results': [{key: val for key, val in row.items() if key != 'average_ms'}
                        for row in value['results']]}


def violations(value):
    limits = load(Path(__file__).with_name('study_limits.json'))
    errors = []
    for row in value['results']:
        limit = limits[row['scene']]
        for key, high in limit.get('maximum', {}).items():
            if row[key] > high:
                errors.append(f"{row['scene']}: {key} > {high}")
        for key, low in limit.get('minimum', {}).items():
            if row[key] < low:
                errors.append(f"{row['scene']}: {key} < {low}")
        if 'support_weight' in limit:
            ratio = row['cached_support_force_mean'] / limit['support_weight']
            if not 0.95 <= ratio <= 1.05:
                errors.append(f"{row['scene']}: support ratio {ratio}")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeat', type=bounded(1, 20), default=5)
    parser.add_argument('--sleep', choices=('off', 'on'), default='off')
    parser.add_argument('--quality', action='store_true')
    args = parser.parse_args()
    try:
        args.output.mkdir(parents=True, exist_ok=True)
        command = [str(args.executable.resolve()), '--sleep', args.sleep]
        values = []
        for i in range(args.repeat):
            run = subprocess.run(command, capture_output=True, text=True, timeout=600)
            (args.output / f'run-{i+1}.json').write_text(run.stdout)
            (args.output / f'run-{i+1}.stderr.txt').write_text(run.stderr)
            require(run.returncode == 0, f'exit {run.returncode}: {run.stderr}')
            value = validate(decode(run.stdout))
            require(value['sleep_enabled'] == (args.sleep == 'on'), 'sleep setting mismatch')
            if values:
                require(deterministic(value) == deterministic(values[0]), 'determinism mismatch')
            values.append(value)
        summary = {'command': command, 'repetitions': args.repeat,
                   'metadata': {k: values[0][k] for k in META},
                   'violations': violations(values[0]), 'scenes': []}
        for j, name in enumerate(SCENES):
            times = [v['results'][j]['average_ms'] for v in values]
            summary['scenes'].append({'scene': name, 'average_ms_runs': times,
                                     'median_ms': statistics.median(times),
                                     'minimum_ms': min(times), 'maximum_ms': max(times)})
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
        require(not args.quality or not summary['violations'], str(summary['violations']))
        return 0
    except (ValueError, TypeError, KeyError, OSError, subprocess.TimeoutExpired) as error:
        print(f'solver study: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
