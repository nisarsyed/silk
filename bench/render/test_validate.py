#!/usr/bin/env python3
"""Mutation-check the independent auditor against preserved real browser runs."""
import copy
import json
from pathlib import Path
import tempfile
import subprocess
import sys
import validate as audit


def rejects(report, mutate):
    changed = copy.deepcopy(report)
    mutate(changed)
    try:
        audit.validate(changed)
    except (ValueError, KeyError, TypeError, IndexError):
        return
    raise AssertionError('corrupt report accepted')


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else 'build/reports/render-frozen-runner-release')
    checked = 0
    for frozen in (False, True):
        for candidate in ('canvas', 'webgl', 'raylib'):
            for diagnostic in (False, True):
                path = root / f'{"frozen-" if frozen else ""}{candidate}-{"diagnostic" if diagnostic else "base"}.json'
                report = audit.load(path)
                result = audit.validate(report)
                assert result['rawTimingIntegrity'] == 'passed' and result['acceptanceEligible'] is False
                assert result['correctnessOnly'] and result['baseBudgetsApplicable'] == (not diagnostic)
                for mutate in (
                    lambda r: r.update(acceptanceEligible=True),
                    lambda r: r.update(status='failed'),
                    lambda r: r.update(schema=True),
                    lambda r: r['configuration'].update(durationSeconds=60),
                    lambda r: r['configuration'].update(width=640),
                    lambda r: r['clock'].update(simulationEnabled=frozen),
                    lambda r: r['calibrationIntervals'].pop(),
                    lambda r: r['initial'].update(steps=1),
                    lambda r: r['final'].update(finiteState=False),
                    lambda r: r['final'].update(steps=r['final']['steps']+1),
                    lambda r: r['frames'].update(count=True),
                    lambda r: r['frames'].update(bytes=1),
                    lambda r: r['frames']['numberNames'].reverse(),
                    lambda r: r['frames']['numbers'].__setitem__(0, float('nan')),
                    lambda r: r['frames']['numbers'].__setitem__(1, r['frames']['numbers'][3]+1),
                    lambda r: r['frames']['numbers'].__setitem__(12, audit.DT),
                    lambda r: r['frames']['numbers'].__setitem__(8, 123456),
                    lambda r: r['frames']['known'].__setitem__(0, 0),
                    lambda r: r['frames']['known'].__setitem__(0, r['frames']['known'][0] ^ (1 << 14)),
                    lambda r: r['frames']['finished'].__setitem__(0, 0),
                    lambda r: r['frames']['work'].__setitem__(0, '00'),
                    lambda r: r['frames']['work'].__setitem__(0, str(2**64)),
                    lambda r: r['frames']['stats'].__setitem__(24, 9),
                    lambda r: r['summary']['cpuMs'].update(p95=123456),
                    lambda r: r['summary'].update(missedTargetSlots=123),
                    lambda r: r['windows'][0].update(executedSteps=123456),
                    lambda r: r['windows'][0].update(contactDrops='1'),
                    lambda r: r['windows'][0].update(gpuUnavailableFrames=123456),
                    lambda r: r['windows'][0].update(coverageReached=False),
                    lambda r: r['gpu']['counts'].__setitem__(0, 1),
                ):
                    rejects(report, mutate)
                if candidate != 'canvas':
                    rejects(report, lambda r: r['glCalls'].update(totalUploadBytes=r['glCalls']['totalUploadBytes']+1))
                if frozen:
                    rejects(report, lambda r: r['frames']['numbers'].__setitem__(6, 1))
                checked += 1
    # A synthetic resolved sample exercises optional GPU distributions even on
    # CI hosts without timer support. This is never saved as device evidence.
    timed = audit.load(root / 'webgl-base.json')
    timed['testSynthetic'] = True
    timed['gpu'].update(supported=True, counterBits=64, unavailableReason=None)
    previous = timed['gpu']['statuses'][0]
    timed['gpu']['statuses'][0] = 2
    timed['gpu']['counts'][previous] -= 1
    timed['gpu']['counts'][2] += 1
    timed['frames']['known'][0] |= 1 << 14
    timed['frames']['numbers'][14] = .5
    for window in timed['windows']:
        samples = [timed['frames']['numbers'][24*i+14] for i in range(window['firstRow'], window['lastRow']+1)
                   if timed['frames']['known'][i] & (1 << 14)]
        window['gpuMs'] = audit.distribution(samples)
        window['gpuUnavailableFrames'] = window['completeFrames'] - len(samples)
    assert audit.validate(timed)['rawTimingIntegrity'] == 'passed'
    rejects(timed, lambda r: r['gpu'].update(counterBits=None))
    rejects(timed, lambda r: r['windows'][0].update(gpuMs=None))
    for reason in ('abort', 'context', 'hidden'):
        report = audit.load(root / f'interrupted-{reason}.json')['report']
        rejects(report, lambda r: None)
    # Exercise the real JS producer across window boundaries, missing windows
    # and final overshoot. The auditor independently checks its output.
    build = sys.argv[2] if len(sys.argv) > 2 else 'build/render-study'
    fixture = json.loads(subprocess.check_output(['node', str(Path(__file__).with_name('test_audit_windows.mjs')),
                         str(root / 'frozen-canvas-base.json'), build], text=True))
    result = audit.validate(fixture)
    assert result['acceptanceEligible'] is False and not result['wholeRun']['cadence']
    assert fixture['windows'][1]['firstRow'] == 2
    assert fixture['windows'][1]['submissionGapMs']['max'] == 9950
    assert fixture['windows'][3]['coverageReached'] is False
    assert result['windows'][3] is None
    assert fixture['windows'][-1]['observedLastSeconds'] == 60.03
    assert all(window['executedSteps'] == 0 for window in fixture['windows'])
    rejects(fixture, lambda r: r['windows'][1]['submissionGapMs'].update(max=60))
    tails = json.loads(subprocess.check_output(['node', str(Path(__file__).with_name('test_audit_windows.mjs')),
                       str(root / 'frozen-canvas-base.json'), build, 'tails'], text=True))
    result = audit.validate(tails)
    assert result['wholeRun']['cpu'] and not result['windows'][1]['cpu']
    assert tails['summary']['cpuMs']['p95'] == 4 and tails['windows'][1]['cpuMs']['p95'] == 40
    # Pin exact inclusive budgets, without using recomputation tolerance to
    # forgive a failing tail. GPU timing cannot substitute for CPU timing.
    summary = dict(cpuMs=dict(p95=16, p99=20), submissionGapMs=dict(p95=25, p99=41, max=100), missedTargetFraction=.01)
    assert all(audit.budgets(summary, 20, 0, 0, 0).values())
    for group, key, threshold in [('cpuMs', 'p95', 16), ('cpuMs', 'p99', 20),
                                  ('submissionGapMs', 'p95', 25), ('submissionGapMs', 'p99', 41),
                                  ('submissionGapMs', 'max', 100)]:
        changed = copy.deepcopy(summary)
        changed[group][key] = threshold + 1e-10
        assert not all(audit.budgets(changed, 20, 0, 0, 0).values())
    for drops, contacts, debt in [(1e-30, 0, 0), (0, 1, 0), (0, 0, audit.DT)]:
        assert not audit.budgets(summary, 20, drops, contacts, debt)['continuity']
    summary['missedTargetFraction'] = .0100000001
    assert not audit.budgets(summary, 20, 0, 0, 0)['missedSlots']
    assert audit.distribution([4, 1, 2, 3]) == dict(count=4, p50=2, p95=4, p99=4, max=4)
    assert audit.cadence([25, 15], 10)['missedTargetSlots'] == 3
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'bad.json'
        for value in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}'):
            path.write_text(value)
            try:
                audit.load(path)
            except ValueError:
                continue
            raise AssertionError('invalid JSON accepted')
    print(f'Collection audit: {checked} real reports, corruption rejection, exact budget boundaries, canonical counters and failed-run exclusion PASS')


if __name__ == '__main__':
    main()
