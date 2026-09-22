#!/usr/bin/env python3
"""Independently audit completed collection records; never certify device acceptance."""
import argparse
import json
import math
from pathlib import Path
import re
import struct
import sys

NUMBERS = 'rafMs callbackMs targetMs submissionMs endMs stepMs snapshotMs prepareMs drawMs counterMs hudMs recordMs debtSeconds droppedSeconds gpuMs uploadBytes poseCopyBytes posePrepareBytes diagnosticCopyBytes diagnosticPrepareBytes gpuBytes drawCalls submissionCalls gpuPollMs'.split()
STATS = 'awakeDynamicCount sleepingDynamicCount bodyCount bodyCapacity contactCount contactCapacity jointCount jointCapacity pairCount pairCapacity bodyCountHigh contactCountHigh jointCountHigh dynamicBodyCount kinematicBodyCount contactConstraintCount jointConstraintCount islandExecutedCount islandSkippedCount islandCount islandBodyCountMax substepCount lastContactDrops allocatorUsedBytes completedSteps'.split()
WORK = 'treeNodeVisits pairCandidates pairProbes wakeVisits bodyWakes bodySleeps graphBodyVisits graphConstraintVisits graphParentProbes proxyCreates proxyDestroys proxyMoves contactDrops'.split()
GPU = 'not-attempted pending resolved pool-full disjoint invalid-result unsupported aborted drain-timeout'.split()
DT = struct.unpack('f', struct.pack('f', 1 / 60))[0]
# The bounded 72,002-row recorder can serialize almost 150 MiB of maximal
# decimal counters plus timing columns. This is an offline input-file bound.
MAX_BYTES = 256 * 1024 * 1024


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value, low=0, high=math.inf, integer=False):
    require(type(value) in (int, float) and math.isfinite(value) and low <= value <= high
            and (not integer or type(value) is int), 'invalid numeric field')
    return value


def decimal(value):
    require(type(value) is str and re.fullmatch(r'0|[1-9][0-9]{0,19}', value), 'noncanonical uint64')
    result = int(value)
    require(result < 2**64, 'uint64 overflow')
    return result


def unique(pairs):
    out = {}
    for key, value in pairs:
        require(key not in out, 'duplicate JSON key: ' + key)
        out[key] = value
    return out


def load(path):
    with Path(path).open('rb') as stream:
        raw = stream.read(MAX_BYTES + 1)
    require(len(raw) <= MAX_BYTES, 'report exceeds 256 MiB')
    return json.loads(raw, object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite JSON ' + value)))


def equal(actual, expected, label):
    """Only recomputation rounding gets tolerance; budget comparisons are exact."""
    if type(expected) is dict:
        require(type(actual) is dict and actual.keys() == expected.keys(), label + ': fields differ')
        for key, value in expected.items():
            equal(actual[key], value, label + '.' + key)
    elif type(expected) is float:
        number(actual)
        require(math.isclose(actual, expected, rel_tol=1e-12, abs_tol=1e-9), label + ': arithmetic differs')
    else:
        require(type(actual) is type(expected) and actual == expected, label + ': differs')


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    return dict(count=len(values), p50=ordered[math.ceil(.5 * len(values)) - 1],
                p95=ordered[math.ceil(.95 * len(values)) - 1],
                p99=ordered[math.ceil(.99 * len(values)) - 1], max=ordered[-1])


def cadence(gaps, period):
    # JS Math.round for nonnegative values; Python round uses ties-to-even.
    missed = sum(max(0, math.floor(gap / period + .5) - 1) for gap in gaps)
    require(missed <= 2**53-1, 'cadence exceeds exact JS integer range')
    return dict(submissionGapMs=distribution(gaps), missedTargetSlots=missed,
                submittedIntervals=len(gaps), missedTargetFraction=missed / (len(gaps) + missed) if gaps else None)


def budgets(summary, period, dropped, contacts, debt):
    cpu, gaps = summary['cpuMs'], summary['submissionGapMs']
    return dict(continuity=dropped == 0 and contacts == 0 and 0 <= debt < DT,
                cpu=cpu is not None and cpu['p95'] <= .8 * period and cpu['p99'] <= period,
                cadence=gaps is not None and gaps['p95'] <= 1.25 * period and
                gaps['p99'] <= 2 * period + 1 and gaps['max'] <= 100,
                missedSlots=summary['missedTargetFraction'] is not None and summary['missedTargetFraction'] <= .01)


def validate(report):
    require(type(report['schema']) is int and report['schema'] == 1 and report['kind'] in ('correctness-only', 'study-collection'), 'unsupported report')
    require(report['status'] == 'collected' and report['failure'] is None and report['phase'] == 'complete',
            'collection failed or incomplete; preserve its partial record')
    require(report['acceptanceEligible'] is False, 'collection primitive cannot certify acceptance')
    cfg = report['configuration']
    frozen = cfg['profile'] == 'render-only'
    require(cfg['profile'] in ('render-only', 'end-to-end') and cfg['candidate'] in ('canvas', 'webgl', 'raylib'), 'invalid profile')
    require(cfg['scene'] in ('pyramid', 'rain', 'chains') and type(cfg['copies']) is int and cfg['copies'] in (1, 2, 4, 8, 16), 'invalid workload')
    require(all(type(cfg[k]) is bool for k in ('sleep', 'diagnostic', 'sustained', 'worker')) and not cfg['worker'], 'invalid collection booleans')
    require((frozen and cfg['instances'] in [2**i for i in range(8, 17)] and cfg['scene'] == 'rain'
             and cfg['copies'] == 1 and not cfg['sleep'] and not cfg['sustained']) or
            (not frozen and cfg['instances'] is None), 'invalid render tier')
    require(not cfg['sustained'] or (cfg['layout'] == 'mobile' and cfg['scene'] == 'rain' and
            cfg['copies'] == 1 and not cfg['sleep'] and not cfg['diagnostic']), 'invalid sustained profile')
    require(cfg['layout'] in ('desktop', 'mobile'), 'invalid layout')
    dimensions = [cfg[k] for k in ('width', 'height', 'cssWidth', 'cssHeight', 'renderDpr')]
    require(dimensions == ([1280, 720, 1280, 720, 1] if cfg['layout'] == 'desktop' else [720, 1280, 360, 640, 2]), 'layout differs from contract')
    duration = number(cfg['durationSeconds'], .25, 300)
    require((report['kind'] == 'correctness-only' and duration <= 5) or
            (report['kind'] == 'study-collection' and duration == (300 if cfg['sustained'] else 60)), 'wrong duration')
    budget = number(cfg['memoryBytes'], (64 if cfg['candidate'] == 'raylib' else 2)*1024**2, 512*1024**2, True)
    require(budget % 65536 == 0 and report['moduleMemory']['linearMemoryBytes'] == budget, 'memory budget differs')
    require(report['clock']['unit'] == 'milliseconds' and report['clock']['simulationEnabled'] is (not frozen), 'clock mismatch')
    number(report['clock']['timeOrigin'])
    intervals = report['calibrationIntervals']
    require(type(intervals) is list and len(intervals) == 240, 'incomplete calibration')
    for value in intervals:
        number(value, 1e-30)
    refresh = sorted(intervals)[119]
    divisor = max(1, math.floor((1000 / 60) / refresh + .5))
    period = refresh * divisor
    require(59 <= 1000 / period <= 61, 'unsupported 60 Hz calibration')
    equal(report['calibration'], dict(intervalCount=240, refreshPeriodMs=refresh, targetPeriodMs=period,
                                    divisor=divisor, effectiveHz=1000 / period, supports60Hz=True), 'calibration')
    initial, final = report['initial'], report['final']
    require(initial['configuration'] == final['configuration'], 'world configuration changed')
    require(initial['finiteState'] is True and final['finiteState'] is True, 'nonfinite simulation')
    number(initial['steps'], 0, 21600, True)
    number(final['steps'], initial['steps'], 21600, True)
    require(initial['steps'] == (120 if frozen else 0), 'wrong initial step baseline')
    require(initial['configuration']['timestep'] == DT and initial['configuration']['substeps'] == 4, 'simulation tuning changed')
    require(initial['configuration']['scene'] == cfg['scene'] and initial['configuration']['copies'] == cfg['copies']
            and initial['configuration']['sleepEnabled'] is cfg['sleep'], 'world/profile mismatch')
    initial_drops, final_drops = decimal(initial['drops']), decimal(final['drops'])
    require(report['rendererRecreatedAfterWarmup'] is False, 'warmed graphics discarded')
    warm = report['warmup']
    require(warm['preparationSteps'] == (120 if frozen else 0), 'wrong preparation baseline')
    require(cfg['sustained'] or (warm['callbacks'] == 120 and warm['world']['steps'] == 120), 'wrong warmup')
    require(not cfg['sustained'] or warm['elapsedMs'] >= 60000, 'short sustained warmup')
    frame = report['frames']
    require(frame['schema'] == 1 and frame['kind'] == 'frame-records', 'unsupported frame schema')
    require(frame['numberNames'] == NUMBERS and frame['statNames'] == STATS and frame['workNames'] == WORK
            and frame['workOrder'] == ['lastStep', 'cumulative'], 'unknown columns')
    count = number(frame['count'], 2, 72002, True)
    capacity = number(frame['capacity'], count, 72002, True)
    require(capacity == math.ceil(duration * 1000 / period) + 2 and frame['bytes'] == 505 * capacity, 'record capacity differs')
    require(frame['pendingFrame'] is False, 'unfinished frame')
    for key, columns in [('numbers', 24), ('stats', 25), ('work', 26), ('known', 1), ('finished', 1)]:
        require(type(frame[key]) is list and len(frame[key]) == count * columns, 'invalid ' + key + ' length')
    gpu = report['gpu']
    require(gpu['statusNames'] == GPU and gpu['attempts'] == count and gpu['pending'] == 0 and
            gpu['frameCapacity'] == capacity and len(gpu['statuses']) == count, 'GPU record dimensions differ')
    require(type(gpu['supported']) is bool, 'invalid GPU support')
    require(gpu['queryCapacity'] == math.ceil(100 / period)+2 and gpu['typedBytes'] == capacity+20*gpu['queryCapacity'], 'GPU allocation dimensions differ')
    if gpu['supported']:
        number(gpu['counterBits'], 30, 64, True)
        require(gpu['unavailableReason'] is None and cfg['candidate'] != 'canvas', 'GPU support claim differs')
    else:
        require(gpu['unavailableReason'] in ('no-webgl-context', 'extension-unavailable', 'counter-bits-unavailable',
                'timer-query-already-active', 'query-allocation-failed'), 'missing GPU unavailability reason')
    totals = [0] * len(GPU)
    cpu, gaps = [], []
    rows = []
    prior_steps, prior_drops, prior_contact = initial['steps'], 0, initial_drops
    prior_work = [decimal(initial['cumulative'][name]) for name in WORK]
    uploads = draws = 0
    for i in range(count):
        n = frame['numbers'][24*i:24*(i+1)]
        s = frame['stats'][25*i:25*(i+1)]
        w = [decimal(v) for v in frame['work'][26*i:26*(i+1)]]
        known = number(frame['known'][i], 0, (1 << 24)-1, True)
        required = ((1 << 14)-1) | (1 << 23)
        require(known & required == required and type(frame['finished'][i]) is int and frame['finished'][i] == 1, 'unfinished required metrics')
        for j, value in enumerate(n):
            number(value, high=2**53-1 if 15 <= j <= 22 else math.inf, integer=15 <= j <= 22)
            require(known & (1 << j) or value == 0, 'unavailable metric contains a value')
        for value in s:
            number(value, 0, 2**32-1, True)
        # rAF and performance.now expose separately converted timestamps. CI
        # recorded 6091.7 versus 6091.6999999999825 for the same instant.
        # Apply only the existing 1e-9 ms arithmetic allowance to that cross-
        # API ordering check. All performance.now ordering and budget gates
        # remain exact; this never alters samples or measured durations.
        require((n[0] <= n[1] or math.isclose(n[0], n[1], rel_tol=0, abs_tol=1e-9)) and
                n[1] <= n[3] <= n[4] and n[11] <= n[4]-n[1], 'invalid frame clocks')
        require(sum(n[5:12]) + n[23] <= n[4]-n[1]+1e-9, 'stage timings exceed the critical path')
        require(s[24] >= prior_steps and s[24]-prior_steps <= 8 and n[13] >= prior_drops and
                w[25] >= prior_contact and all(w[13+j] >= prior_work[j] for j in range(13)), 'counters moved backwards or step cap exceeded')
        require([s[3], s[5], s[7]] == [initial['configuration'][k] for k in ('bodyCapacity', 'contactCapacity', 'jointCapacity')], 'world capacities differ')
        require(s[2] <= s[3] and s[4] <= s[5] and s[6] <= s[7] and s[8] <= s[9] and s[23] <= budget, 'capacity exceeded')
        elapsed = (n[1]-frame['numbers'][1]) / 1000
        require(elapsed >= 0 and n[12] < DT, 'invalid debt or elapsed time')
        if i == 0:
            require(s[24] == initial['steps'] and n[12] == n[13] == 0 and w[25] == initial_drops, 'first frame is not the initial state')
        if frozen:
            require(s[24] == 120 and n[12] == n[13] == n[6] == n[7] == 0, 'frozen source changed')
        else:
            equal((s[24]-initial['steps'])*DT+n[12]+n[13], elapsed, 'step/time accounting')
        status = number(gpu['statuses'][i], 2, 8, True)
        totals[status] += 1
        require(bool(known & (1 << 14)) == (status == 2), 'GPU availability differs')
        require(gpu['supported'] or status == 6, 'unsupported GPU reports samples')
        if cfg['candidate'] != 'canvas':
            require(known & ((1 << 15) | (1 << 21)) == ((1 << 15) | (1 << 21)), 'missing observed GL metrics')
            require(n[21] > 0, 'missing GL submission')
            uploads += n[15]
            draws += n[21]
        if cfg['candidate'] == 'raylib':
            require(n[16] == n[18] == 0, 'co-located JS boundary copies')
        if frozen and cfg['candidate'] == 'webgl':
            require(n[15] == 0, 'frozen WebGL upload')
        if i:
            require(n[0] > rows[-1]['n'][0] and n[1] >= rows[-1]['n'][1] and n[3] >= rows[-1]['n'][3], 'clock moved backwards')
            gaps.append(n[3]-rows[-1]['n'][3])
        cpu.append(n[4]-n[1])
        rows.append(dict(n=n, s=s, w=w, elapsed=elapsed, cpu=cpu[-1], gap=gaps[-1] if i else None,
                         gpu=n[14] if status == 2 else None, steps=s[24]-prior_steps,
                         drops=n[13]-prior_drops, contacts=w[25]-prior_contact))
        prior_steps, prior_drops, prior_contact, prior_work = s[24], n[13], w[25], w[13:]
    require(type(gpu['counts']) is list and len(gpu['counts']) == len(GPU), 'GPU totals shape differs')
    for value in gpu['counts']:
        number(value, 0, count, True)
    require(gpu['counts'] == totals, 'GPU status totals differ')
    summary = dict(completeFrames=count, pendingFrame=False, cpuMs=distribution(cpu), **cadence(gaps, period))
    equal(report['summary'], summary, 'summary')
    last = rows[-1]
    require(final['steps'] == last['s'][24] and final_drops == last['w'][25], 'final world counters differ')
    for world, row in ((initial, rows[0]), (final, last)):
        for j, name in enumerate(STATS[:13]):
            number(world['stats'][name], 0, 2**32-1, True)
            require(world['stats'][name] == row['s'][j], 'world statistics differ')
        for j, name in enumerate(STATS[13:22]):
            number(world['last'][name], 0, 2**32-1, True)
            require(world['last'][name] == row['s'][13+j], 'last-step statistics differ')
        for j, name in enumerate(WORK):
            require(decimal(world['work'][name]) == row['w'][j], 'last-step work differs')
    for j, name in enumerate(WORK):
        require(decimal(final['cumulative'][name]) == last['w'][13+j], 'final work differs')
    equal(report['elapsedSeconds'], last['elapsed'], 'elapsed')
    equal(report['debtSeconds'], last['n'][12], 'debt')
    equal(report['droppedSeconds'], last['n'][13], 'drops')
    equal(report['measurementStart'], rows[0]['n'][1], 'measurement start')
    equal(report['measurementEnd'], last['n'][4], 'measurement end')
    require(last['elapsed'] >= duration and rows[-2]['elapsed'] < duration, 'duration boundary differs')
    gl = report['glCalls']
    require((cfg['candidate'] == 'canvas' and gl is None) or
            (cfg['candidate'] != 'canvas' and gl['valid'] is True and gl['sections'] == count and
             gl['totalUploadBytes'] == uploads and gl['totalDrawCalls'] == draws), 'GL totals differ')
    windows = []
    window_gates = []
    for index in range(math.ceil(duration / 10)):
        selected = [(i, r) for i, r in enumerate(rows) if min(math.ceil(duration / 10)-1, math.floor(r['elapsed']/10)) == index]
        end = min(duration, 10*(index+1))
        data = [r for _, r in selected]
        local = dict(index=index, startSeconds=10*index, endSeconds=end,
                     coverageReached=bool(data) and last['elapsed'] >= end, pendingFrame=False,
                     firstRow=selected[0][0] if data else None, lastRow=selected[-1][0] if data else None,
                     completeFrames=len(data), observedFirstSeconds=data[0]['elapsed'] if data else None,
                     observedLastSeconds=data[-1]['elapsed'] if data else None,
                     cpuMs=distribution([r['cpu'] for r in data]), gpuMs=distribution([r['gpu'] for r in data if r['gpu'] is not None]),
                     gpuUnavailableFrames=sum(r['gpu'] is None for r in data),
                     **cadence([r['gap'] for r in data if r['gap'] is not None], period),
                     executedSteps=sum(r['steps'] for r in data), droppedSeconds=sum(r['drops'] for r in data),
                     contactDrops=str(sum(r['contacts'] for r in data)), endDebtSeconds=data[-1]['n'][12] if data else None,
                     allocatorBytesMax=max(r['s'][23] for r in data) if data else None)
        windows.append(local)
        window_gates.append(budgets(local, period, local['droppedSeconds'], int(local['contactDrops']), local['endDebtSeconds']) if data else None)
    require(len(report['windows']) == len(windows), 'window count differs')
    for actual, expected in zip(report['windows'], windows):
        equal(actual, expected, 'window')
    return dict(schema=1, kind='collection-timing-audit', rawTimingIntegrity='passed', acceptanceEligible=False,
                baseBudgetsApplicable=not cfg['diagnostic'], correctnessOnly=report['kind'] == 'correctness-only',
                wholeRun=budgets(summary, period, last['n'][13], final_drops, last['n'][12]),
                windows=window_gates, unverified=['provenance authenticity', 'device conditions', 'real input',
                'startup/cache protocol', 'allocation/GC and memory profiling', 'physical presentation',
                'full workload/physical configuration', 'full-quality companion runs', 'five-repeat protocol', 'cross-device renderer selection'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    args = parser.parse_args()
    try:
        result = validate(load(args.report))
    except (ValueError, KeyError, TypeError, IndexError, OverflowError) as error:
        print('Invalid collection: ' + str(error), file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
