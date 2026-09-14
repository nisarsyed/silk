#!/usr/bin/env python3
"""Validate exact-counter WASM reports and compare native-compatible physical results."""
import argparse
import copy
import json
import math
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import report as native

MAX_BYTES = 10 * 1024 * 1024
require, fields, number = native.require, native.fields, native.number

def load(path):
    with Path(path).open('rb') as stream:
        data = stream.read(MAX_BYTES + 1)
    require(len(data) <= MAX_BYTES, 'report exceeds 10 MiB')
    return json.loads(data.decode('utf8'), object_pairs_hook=native.unique_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f'non-finite {value}')))

def text(value, limit, name):
    require(type(value) is str and 0 < len(value) <= limit, f'{name}: invalid string')

def decimal(value):
    require(type(value) is str and re.fullmatch(r'0|[1-9][0-9]{0,19}', value) is not None,
            'uint64 must be a canonical decimal string')
    result = int(value)
    require(result <= 2**64 - 1, 'uint64 overflow')
    return result

def native_report(value):
    result = copy.deepcopy(value['native'])
    for row in result['results']:
        row['drops'] = decimal(row['drops'])
        for work in (row['step']['work'], row['cumulative_work']):
            for name in work:
                work[name] = decimal(work[name])
    return native.validate(result)

def near_report(value, expected, name):
    # Native %.9g / JS toPrecision(9) has at most 5e-9 relative rounding error;
    # the tiny absolute allowance handles sums close to zero, not physical gates.
    require(math.isclose(value, expected, rel_tol=5e-9, abs_tol=1e-12), f'{name}: sample summary mismatch')

def validate(value, quality=False):
    fields(value, ('schema_version', 'kind', 'build', 'runtime', 'profile', 'timing_scope', 'clock',
                   'module', 'regions', 'failures', 'execution_status', 'native'), 'WASM report')
    require(type(value['schema_version']) is int and value['schema_version'] == 1, 'unsupported WASM schema')
    require(value['kind'] == 'silk-wasm-benchmark', 'wrong report kind')
    require(value['execution_status'] == 'complete' and value['failures'] == [],
            'execution failed; inspect the preserved failures and partial state')
    build = value['build']
    fields(build, ('revision', 'dirty', 'source_sha256', 'wasm_sha256', 'compiler', 'mode',
                   'link_flags', 'compile_commands', 'sdk_version', 'sdk_commit'), 'build')
    require(type(build['dirty']) is bool, 'dirty must be boolean')
    for name, length in (('revision', 40), ('sdk_commit', 40), ('source_sha256', 64), ('wasm_sha256', 64)):
        require(type(build[name]) is str and re.fullmatch('[0-9a-f]{'+str(length)+'}', build[name]), f'invalid {name}')
    require(build['sdk_version'] == '6.0.9' and build['sdk_commit'] == '5eb0bde7585670252e8ba05e9d361627bffd08b5', 'SDK baseline changed')
    require(build['mode'] in ('Debug', 'Release'), 'invalid build mode')
    text(build['compiler'], 8192, 'compiler'); text(build['link_flags'], 4096, 'link flags')
    commands = build['compile_commands']
    require(type(commands) is list and 1 <= len(commands) <= 256, 'invalid compilation provenance')
    for command in commands:
        require(type(command) is dict and {'directory', 'file', 'command'} <= set(command) <= {'directory', 'file', 'command', 'output'}, 'invalid compile command')
        for item in command.values():
            text(item, 8192, 'compile command field')
    runtime = value['runtime']
    fields(runtime, ('kind', 'engine', 'device', 'os', 'hardware_concurrency', 'automated',
                     'visibility_start', 'visibility_end', 'process_reused', 'module_per_repeat'), 'runtime')
    require(runtime['kind'] in ('node', 'browser'), 'invalid runtime kind')
    for name in ('engine', 'device', 'os'):
        text(runtime[name], 1024, name)
    number(runtime['hardware_concurrency'], 1, 1024, 'hardware concurrency', True)
    require(type(runtime['automated']) is bool and runtime['process_reused'] is True and runtime['module_per_repeat'] is True, 'invalid run conditions')
    visibility = 'visible' if runtime['kind'] == 'browser' else 'not-applicable'
    require(runtime['visibility_start'] == runtime['visibility_end'] == visibility, 'invalid foreground conditions')
    profile = value['profile']
    fields(profile, ('warmup_steps', 'measured_steps', 'sleep_enabled', 'fixtures', 'settling_profile'), 'profile')
    number(profile['warmup_steps'], 0, 10000, 'warmup', True)
    number(profile['measured_steps'], 1, 10000, 'steps', True)
    require(type(profile['sleep_enabled']) is bool and type(profile['settling_profile']) is bool, 'invalid profile booleans')
    require(profile['settling_profile'] == (profile['warmup_steps'] == 120 and profile['measured_steps'] == 600), 'incorrect settling profile claim')
    require(type(profile['fixtures']) is list and 1 <= len(profile['fixtures']) <= 7 and
            len(set(profile['fixtures'])) == len(profile['fixtures']) and all(x in native.SCENES for x in profile['fixtures']), 'invalid fixture selection')
    text(value['timing_scope'], 1024, 'timing scope')
    clock = value['clock']
    fields(clock, ('pairs', 'minimum_positive_ms', 'average_pair_ms'), 'clock')
    require(type(clock['pairs']) is int and clock['pairs'] == 1000, 'invalid timer probe count')
    number(clock['average_pair_ms'], 0, 86400000, 'timer overhead')
    if clock['minimum_positive_ms'] is not None:
        number(clock['minimum_positive_ms'], 1e-30, 86400000, 'timer granularity')
    module = value['module']
    fields(module, ('memory_bytes', 'load_ms'), 'module')
    number(module['memory_bytes'], 2097152, 536870912, 'module budget', True)
    require(module['memory_bytes'] % 65536 == 0, 'unaligned module budget')
    number(module['load_ms'], 0, 86400000, 'module load')
    base = native_report(value)
    require(base['metadata']['revision'] == build['revision'] + ('-dirty' if build['dirty'] else ''), 'source provenance mismatch')
    require(base['metadata']['flags'] == build['link_flags'] and base['metadata']['build'] == build['mode'], 'compiler provenance mismatch')
    require([row['scene'] for row in base['results']] == profile['fixtures'], 'missing or reordered fixtures')
    regions = value['regions']
    require(type(regions) is list and len(regions) == len(base['results']), 'missing timing regions')
    for row, region in zip(base['results'], regions):
        fields(region, ('scene', 'setup_ms', 'warmup_ms', 'prepare_ms', 'mutation_ms', 'step_ms', 'quality_ms',
                        'report_ms', 'snapshot_ms', 'step_samples_ms', 'mutation_samples_ms', 'memory', 'driver'), 'regions')
        require(region['scene'] == row['scene'], 'region scene mismatch')
        settings = row['settings']
        require(settings['warmup_steps'] == profile['warmup_steps'] and settings['measured_steps'] == profile['measured_steps'] and settings['sleep_enabled'] == profile['sleep_enabled'], 'profile/settings mismatch')
        for name in ('setup_ms', 'warmup_ms', 'prepare_ms', 'mutation_ms', 'step_ms', 'quality_ms', 'report_ms', 'snapshot_ms'):
            number(region[name], 0, 86400000, name)
        steps = profile['measured_steps']
        samples = region['step_samples_ms']; mutations = region['mutation_samples_ms']
        require(type(samples) is list and len(samples) == steps, 'missing step samples')
        require(type(mutations) is list and len(mutations) == (steps if row['scene'] == 'churn' else 0), 'invalid mutation samples')
        for sample in samples + mutations:
            number(sample, 0, 86400000, 'timing sample')
        ordered = sorted(samples)
        timing = row['timing_ms']
        near_report(timing['average'], sum(samples)/steps, 'average')
        near_report(timing['median'], ordered[steps//2] if steps % 2 else sum(ordered[steps//2-1:steps//2+1])/2, 'median')
        near_report(timing['p95'], ordered[math.ceil(0.95*steps)-1], 'p95')
        near_report(timing['max'], ordered[-1], 'max')
        near_report(region['step_ms'], sum(samples), 'step total')
        near_report(region['mutation_ms'], sum(mutations), 'mutation total')
        near_report(timing['mutation_average'], sum(mutations)/steps, 'mutation average')
        driver = region['driver']
        fields(driver, ('index', 'phase', 'failed', 'complete'), 'driver')
        require(driver == {'index':profile['warmup_steps']+steps, 'phase':0, 'failed':False, 'complete':True}, 'driver did not complete exact work')
        require(type(driver['index']) is int and type(driver['phase']) is int and type(driver['failed']) is bool and type(driver['complete']) is bool, 'invalid driver status types')
        memory = region['memory']
        fields(memory, ('linear_memory', 'stack', 'static_end', 'heap_base', 'allocator_used', 'context_bytes', 'adapter_bytes', 'output_bytes', 'context_base_bytes', 'requested_bytes', 'sample_bytes'), 'memory')
        for name, amount in memory.items():
            number(amount, 0, 536870912, name, True)
        b, c, j = (settings[key] for key in ('body_capacity', 'contact_capacity', 'joint_capacity'))
        require(memory['linear_memory'] == module['memory_bytes'] and memory['stack'] == 1048576, 'fixed budget mismatch')
        require(memory['adapter_bytes'] == memory['context_base_bytes']+144*b+136*c+60*j, 'adapter budget mismatch')
        require(memory['output_bytes'] == 132*b+136*c+60*j and memory['sample_bytes'] == 16*steps, 'JS output budget mismatch')
        require(memory['context_bytes'] > memory['context_base_bytes'] > 0, 'missing fixture context')
        requested = row['memory_bytes']['arena'] + memory['context_bytes'] + memory['adapter_bytes'] - memory['context_base_bytes']
        require(memory['requested_bytes'] == requested <= memory['allocator_used'] <= memory['linear_memory']-memory['heap_base'], 'allocator budget mismatch')
        aligned_end = (memory['static_end'] + 15) // 16 * 16
        # Emscripten Debug uses stack-first; Release puts the stack after data.
        # Validate either exact 16-byte-aligned layout, not an assumed ordering.
        stack_first = memory['static_end'] >= memory['stack'] and memory['heap_base'] == aligned_end
        data_first = memory['heap_base'] == aligned_end + memory['stack']
        require(0 < memory['static_end'] <= memory['heap_base'] < memory['linear_memory'] and
                (stack_first or data_first), 'module layout mismatch')
    if quality:
        native.quality_check(base)
    return base

def compare(a, b, cross_build=False):
    left, right = validate(a, cross_build), validate(b, cross_build)
    require(a['profile'] == b['profile'], 'profile mismatch')
    if not cross_build:
        require(a['build']['wasm_sha256'] == b['build']['wasm_sha256'] and a['build']['source_sha256'] == b['build']['source_sha256'], 'same-build comparison requires matching binary/source')
        for x, y in zip(a['regions'], b['regions']):
            for key in ('context_bytes', 'adapter_bytes', 'output_bytes', 'context_base_bytes', 'requested_bytes', 'sample_bytes'):
                require(x['memory'][key] == y['memory'][key], 'deterministic memory mismatch')
    return native.compare(left, right, cross_build)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    check = sub.add_parser('validate'); check.add_argument('report'); check.add_argument('--quality', action='store_true')
    diff = sub.add_parser('compare'); diff.add_argument('before'); diff.add_argument('after'); diff.add_argument('--cross-build', action='store_true')
    cross = sub.add_parser('native'); cross.add_argument('native'); cross.add_argument('wasm')
    args = parser.parse_args()
    try:
        if args.command == 'validate':
            validate(load(args.report), args.quality); print('valid')
        elif args.command == 'compare':
            print(json.dumps(compare(load(args.before), load(args.after), args.cross_build), indent=2))
        else:
            baseline = native.load(args.native); candidate = validate(load(args.wasm), True)
            print(json.dumps(native.compare(baseline, candidate, True), indent=2))
    except (ValueError, KeyError, TypeError, OverflowError, RecursionError) as error:
        print(str(error), file=sys.stderr); sys.exit(1)
