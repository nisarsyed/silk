#!/usr/bin/env python3
"""Exercise missing, reordered, unmatched and failing five-repeat evidence."""
import copy
import json
from pathlib import Path
import tempfile

import protocol


def rows_for(layout):
    rows = []
    next_start = 1_790_000_000_000
    for key in protocol.required_workloads(layout):
        for repeat, rotation in enumerate(protocol.ROTATIONS, 1):
            for candidate in rotation:
                start = next_start
                end = start + (300_000 if key[0] == 'sustained' else 60_000)
                rows.append(dict(path=f'{layout}-{len(rows)}.json', key=key, candidate=candidate,
                                 repeat=repeat, startMs=start, endMs=end, device='reference',
                                 layout=layout, identity=('source', 'contract', 'browser'),
                                 matched={'memoryBytes': 64*1024*1024},
                                 audit={'wholeRun': {'continuity': True, 'cpu': True,
                                                     'cadence': True, 'missedSlots': True},
                                        'windows': [{'continuity': True, 'cpu': True,
                                                     'cadence': True, 'missedSlots': True}],
                                        'input': {'minimumSamplesObserved': True}}))
                next_start = end + 300_000
    return rows


def main():
    desktop = rows_for('desktop')
    mobile = rows_for('mobile')
    for layout, rows, count in (('desktop', desktop, 1215), ('mobile', mobile, 1230)):
        result = protocol.analyze(rows, [], layout, 'reference')
        assert result['expectedSuccessfulRuns'] == count and result['successfulSlots'] == count
        assert result['protocolComplete'] and result['numericalGatesPassed']
        assert not result['acceptanceEligible'] and result['unverified']

    missing = protocol.analyze(desktop[:-1], [], 'desktop', 'reference')
    assert not missing['protocolComplete'] and missing['successfulSlots'] == 1214
    assert missing['missingSlots'][0]['candidates'] == ['canvas']

    reordered = copy.deepcopy(desktop)
    reordered[0]['startMs'], reordered[1]['startMs'] = reordered[1]['startMs'], reordered[0]['startMs']
    assert protocol.analyze(reordered, [], 'desktop', 'reference')['candidateOrderIssues']

    changed = copy.deepcopy(desktop)
    changed[1]['matched']['memoryBytes'] *= 2
    mismatch = protocol.analyze(changed, [], 'desktop', 'reference')
    assert not mismatch['protocolComplete']
    assert mismatch['comparisonIssues'][0]['reason'] == 'matched workload settings differ'

    duplicate = protocol.analyze(desktop+[desktop[0]], [], 'desktop', 'reference')
    assert not duplicate['protocolComplete'] and duplicate['comparisonIssues'][0]['reason'] == 'duplicate successful candidate slot'

    short_cooldown = copy.deepcopy(desktop)
    short_cooldown[1]['startMs'] = short_cooldown[0]['endMs']+299_999
    assert protocol.analyze(short_cooldown, [], 'desktop', 'reference')['cooldownIssues']

    failing = copy.deepcopy(desktop)
    failing[0]['audit']['wholeRun']['cpu'] = False
    result = protocol.analyze(failing, [], 'desktop', 'reference')
    assert result['protocolComplete'] and not result['numericalGatesPassed']
    assert result['baseGateFailures'] == [{'path': failing[0]['path'],
                                           'failures': [{'scope': 'wholeRun', 'metrics': ['cpu']}]}]
    empty_window = copy.deepcopy(desktop)
    empty_window[0]['audit']['windows'][0] = None
    result = protocol.analyze(empty_window, [], 'desktop', 'reference')
    assert result['baseGateFailures'][0]['failures'] == [
        {'scope': 'window', 'window': 1, 'metrics': ['no complete frames']}]

    short_input = copy.deepcopy(desktop)
    interaction = next(row for row in short_input if row['key'][0] == 'interaction')
    interaction['audit']['input']['minimumSamplesObserved'] = False
    result = protocol.analyze(short_input, [], 'desktop', 'reference')
    assert result['protocolComplete'] and not result['inputMinimumsObserved']
    assert result['interactionMinimumShortfalls'] == [interaction['path']]

    rejected = protocol.analyze(desktop, [{'path': 'failed.json', 'reason': 'partial run'}],
                                'desktop', 'reference')
    assert rejected['protocolComplete'] and rejected['rejectedRecords']
    retained_failure = protocol.analyze(desktop, [], 'desktop', 'reference',
                                        [{'path': 'disrupted.json', 'phase': 'measurement',
                                          'reason': 'context lost'}])
    assert retained_failure['protocolComplete'] and len(retained_failure['failedRecords']) == 1
    with tempfile.TemporaryDirectory() as directory:
        paths = [Path(directory)/'failed.json', Path(directory)/'invalid.json']
        paths[0].write_text(json.dumps({'schema': 1, 'kind': 'study-collection',
                                        'status': 'failed', 'phase': 'measurement',
                                        'failure': {'phase': 'measurement', 'reason': 'context lost'},
                                        'collectionConditions': {'repeat': 1}}))
        paths[1].write_text('{bad json')
        valid, failed, rejected = protocol.audit_files(paths)
        assert not valid and len(failed) == 1 and len(rejected) == 1
    print('Renderer five-repeat protocol mutations passed')


if __name__ == '__main__':
    main()
