#!/usr/bin/env python3
"""Audit a renderer study's reported five-repeat matrix without certifying a device."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import validate as timing

CANDIDATES = ('canvas', 'webgl', 'raylib')
ROTATIONS = (CANDIDATES, ('webgl', 'raylib', 'canvas'),
             ('raylib', 'canvas', 'webgl'), ('canvas', 'raylib', 'webgl'),
             ('raylib', 'webgl', 'canvas'))
TIERS = tuple(2**power for power in range(8, 17))


def workload(profile, scene, copies=1, sleep=False, instances=None):
    return (profile, scene, copies, sleep, instances)


def required_workloads(layout):
    """Every required browser profile; the copies=1 rows are the defaults."""
    timing.require(layout in ('desktop', 'mobile'), 'invalid target layout')
    rows = [workload(profile, scene, copies, sleep)
            for profile in ('base', 'diagnostic')
            for scene in ('pyramid', 'rain', 'chains')
            for copies in (1, 2, 4, 8, 16)
            for sleep in (False, True)]
    rows += [workload(profile, 'rain', instances=tier)
             for profile in ('frozen', 'frozen-diagnostic') for tier in TIERS]
    rows += [workload('interaction', scene) for scene in ('pyramid', 'rain', 'chains')]
    if layout == 'mobile':
        rows.append(workload('sustained', 'rain'))
    return rows


def profile_of(configuration):
    frozen = configuration['profile'] == 'render-only'
    if frozen:
        return 'frozen-diagnostic' if configuration['diagnostic'] else 'frozen'
    if configuration['interaction']:
        return 'interaction'
    if configuration['sustained']:
        return 'sustained'
    return 'diagnostic' if configuration['diagnostic'] else 'base'


def key_of(configuration):
    return workload(profile_of(configuration), configuration['scene'], configuration['copies'],
                    configuration['sleep'], configuration['instances'])


def label(key):
    profile, scene, copies, sleep, instances = key
    if instances is not None:
        return f'{profile}/{scene}/{instances}'
    if profile in ('interaction', 'sustained'):
        return f'{profile}/{scene}'
    return f'{profile}/{scene}/{copies}x/sleep-{"on" if sleep else "off"}'


def recorded_start(value):
    timing.require(type(value) is str and value.endswith('Z'), 'missing UTC collection start')
    instant = datetime.fromisoformat(value.replace('Z', '+00:00'))
    timing.require(instant.tzinfo == timezone.utc, 'collection start is not UTC')
    return instant.timestamp() * 1000


def entry(path, report):
    """Validate one raw run before retaining only small comparison metadata."""
    audit = timing.validate(report)
    timing.require(report['kind'] == 'study-collection' and not audit['syntheticTiming'],
                   'short or synthetic run cannot enter the protocol')
    configuration = report['configuration']
    conditions = report['collectionConditions']
    timing.require(type(conditions) is dict and conditions['source'] == 'operator-entered',
                   'missing operator conditions')
    profile = profile_of(configuration)
    timing.require(conditions['profile'] == profile, 'operator profile differs from run')
    repeat = timing.number(conditions['repeat'], 1, 5, True)
    candidate = configuration['candidate']
    timing.require(conditions['candidateOrder'] == list(ROTATIONS[repeat-1]) and
                   type(conditions['candidateSlot']) is int and
                   conditions['candidateSlot'] == ROTATIONS[repeat-1].index(candidate)+1,
                   'candidate rotation stamp differs')
    start_ms = recorded_start(conditions['recordedAtIso'])
    end_ms = timing.number(report['clock']['timeOrigin']) + timing.number(report['measurementEnd'])
    timing.require(end_ms >= start_ms, 'measurement ends before recorded start')
    device = conditions['deviceLabel']
    timing.require(type(device) is str and 0 < len(device.strip()) <= 80, 'missing device label')
    user_agent = report['browser']['userAgent']
    timing.require(type(user_agent) is str and user_agent and 'Headless' not in user_agent,
                   'headless browser is not physical evidence')
    layout = configuration['layout']
    timing.require(conditions['power'] == ('plugged' if layout == 'desktop' else 'battery'),
                   'power setting differs from protocol')
    battery = conditions['batteryPercent']
    if layout == 'mobile':
        timing.number(battery, 50, 90)
    elif battery is not None:
        timing.number(battery, 0, 100)
    timing.require(conditions['brightnessPercent'] == 50 and
                   conditions['lowPowerMode'] is False, 'brightness or low-power setting differs')
    timing.number(conditions['ambientCelsius'], -30, 60)
    timing.require(conditions['thermalState'] in ('normal', 'unknown'), 'warm/hot thermal start')
    display = conditions['display']
    timing.number(display['screenWidth'], 1)
    timing.number(display['screenHeight'], 1)
    timing.require(display['deviceDpr'] == configuration['deviceDpr'], 'reported DPR differs')
    provenance = report['provenance']
    source = provenance['source']
    contract = provenance['contract']
    timing.require(source['dirty'] is False, 'dirty build cannot enter matched protocol')
    timing.require(contract['declaredRevision'] == 2 and
                   provenance['verification'] == 'assembly-time', 'unrecognized provenance')
    fingerprint = hashlib.sha256(json.dumps(provenance, sort_keys=True,
                                            separators=(',', ':')).encode()).hexdigest()
    identity = (fingerprint, user_agent)
    # Candidate is the sole allowed configuration difference in a matched run.
    matched = {key: value for key, value in configuration.items() if key != 'candidate'}
    return dict(path=str(path), key=key_of(configuration), candidate=candidate,
                repeat=repeat, startMs=start_ms, endMs=end_ms, device=device.strip(),
                layout=layout, identity=identity, matched=matched, audit=audit)


def audit_files(paths):
    valid, failed, rejected = [], [], []
    for path in paths:
        try:
            report = timing.load(path)
            if (type(report) is dict and type(report.get('schema')) is int and
                    report['schema'] == 1 and report.get('kind') == 'study-collection' and
                    report.get('status') == 'failed' and type(report.get('failure')) is dict and
                    type(report.get('collectionConditions')) is dict):
                failed.append(dict(path=str(path), phase=report['failure'].get('phase'),
                                   reason=report['failure'].get('reason')))
                continue
            valid.append(entry(path, report))
        except (OSError, ValueError, KeyError, TypeError, IndexError, OverflowError) as error:
            rejected.append(dict(path=str(path), reason=str(error)))
    return valid, failed, rejected


def failed_metrics(gate):
    if gate is None:
        return ['no complete frames']
    return [name for name, passed in gate.items() if passed is not True]


def analyze(rows, rejected, layout, device, failed=()):
    """Analyze already validated runs. Protocol completeness never means acceptance."""
    required = required_workloads(layout)
    expected = set(required)
    issues = []
    slots = {}
    reference_identity = None
    reference_settings = {}
    for row in rows:
        path = row['path']
        key = row['key']
        if row['device'] != device or row['layout'] != layout or key not in expected:
            issues.append(dict(path=path, reason='wrong device, layout, or workload'))
            continue
        if reference_identity is None:
            reference_identity = row['identity']
        elif row['identity'] != reference_identity:
            issues.append(dict(path=path, reason='build, contract, or browser differs'))
            continue
        if key not in reference_settings:
            reference_settings[key] = row['matched']
        elif row['matched'] != reference_settings[key]:
            issues.append(dict(path=path, reason='matched workload settings differ'))
            continue
        slot = (key, row['repeat'], row['candidate'])
        if slot in slots:
            issues.append(dict(path=path, reason='duplicate successful candidate slot'))
            continue
        slots[slot] = row
    order_issues = []
    missing = []
    gate_failures = []
    interaction_shortfalls = []
    for key in required:
        for repeat, rotation in enumerate(ROTATIONS, 1):
            observed = [slots[(key, repeat, candidate)] for candidate in rotation
                        if (key, repeat, candidate) in slots]
            if ([row['candidate'] for row in sorted(observed, key=lambda row: row['startMs'])] !=
                    [row['candidate'] for row in observed]):
                order_issues.append(dict(workload=label(key), repeat=repeat,
                                         reason='candidate start order differs'))
            absent = [candidate for candidate in rotation if (key, repeat, candidate) not in slots]
            if absent:
                missing.append(dict(workload=label(key), repeat=repeat, candidates=absent))
            for row in observed:
                if key[0] in ('base', 'sustained'):
                    failures = []
                    metrics = failed_metrics(row['audit']['wholeRun'])
                    if metrics:
                        failures.append(dict(scope='wholeRun', metrics=metrics))
                    for index, gate in enumerate(row['audit']['windows'], 1):
                        metrics = failed_metrics(gate)
                        if metrics:
                            failures.append(dict(scope='window', window=index, metrics=metrics))
                    if failures:
                        gate_failures.append(dict(path=row['path'], failures=failures))
                if key[0] == 'interaction' and not row['audit']['input']['minimumSamplesObserved']:
                    interaction_shortfalls.append(row['path'])
    cooldown_issues = []
    chronological = sorted(slots.values(), key=lambda row: row['startMs'])
    for previous, current in zip(chronological, chronological[1:]):
        if previous['candidate'] != current['candidate']:
            idle_ms = current['startMs'] - previous['endMs']
            if idle_ms < 300000:
                cooldown_issues.append(dict(previous=previous['path'], current=current['path'],
                                            observedIdleSeconds=round(idle_ms/1000, 3)))
    # Disrupted/corrupt attempts remain in the archive, but a valid replacement
    # may fill their slot. Only successful matched runs determine completeness.
    complete = not (issues or order_issues or missing or cooldown_issues)
    return dict(schema=1, kind='renderer-study-protocol-audit', acceptanceEligible=False,
                deviceLabel=device, layout=layout, expectedWorkloads=len(required),
                expectedSuccessfulRuns=len(required)*15, successfulSlots=len(slots),
                protocolComplete=complete, numericalGatesPassed=complete and not gate_failures,
                inputMinimumsObserved=complete and not interaction_shortfalls,
                failedRecords=list(failed), rejectedRecords=rejected,
                comparisonIssues=issues, candidateOrderIssues=order_issues,
                cooldownIssues=cooldown_issues, missingSlots=missing, baseGateFailures=gate_failures,
                interactionMinimumShortfalls=interaction_shortfalls,
                unverified=['operator-entered conditions and physical-device authenticity',
                            'actual five-minute idle state and absence of background work',
                            'visual/physical presentation and graphics backend',
                            'input timestamp precision and latency qualification',
                            'startup/cache, memory/GC profiling, native quality companions',
                            'cross-device renderer selection'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='directory containing raw collector JSON only')
    parser.add_argument('--device-label', required=True)
    parser.add_argument('--layout', required=True, choices=('desktop', 'mobile'))
    args = parser.parse_args()
    if not args.directory.is_dir():
        parser.error('directory does not exist')
    paths = sorted(args.directory.rglob('*.json'))
    rows, failed, rejected = audit_files(paths)
    print(json.dumps(analyze(rows, rejected, args.layout, args.device_label, failed), indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
