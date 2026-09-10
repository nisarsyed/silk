"""Collect five alternating matched release runs from two prebuilt bin directories."""
import argparse
import json
from pathlib import Path
import statistics
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import report
import study

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('before', type=Path)
parser.add_argument('after', type=Path)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
data = {}
commands = []
for mode in ('off', 'on'):
    for index in range(5):
        for label in (('before', 'after') if index % 2 == 0 else ('after', 'before')):
            binaries = getattr(args, label).resolve()
            for kind, executable in (('matrix', 'sl_bench'), ('study', 'sl_bench_study')):
                command = [str(binaries / executable), '--sleep', mode]
                commands.append(command)
                run = subprocess.run(command, capture_output=True, text=True, timeout=600)
                stem = args.output / f'{label}-{kind}-{mode}-{index+1}'
                stem.with_suffix('.json').write_text(run.stdout)
                if run.stderr:
                    stem.with_suffix('.stderr.txt').write_text(run.stderr)
                if run.returncode:
                    raise RuntimeError(f'{command}: {run.returncode}: {run.stderr}')
                value = report.decode(run.stdout)
                if kind == 'matrix':
                    report.validate(value)
                    report.quality_check(value)
                else:
                    study.validate(value)
                    if study.violations(value):
                        raise ValueError(study.violations(value))
                data.setdefault((label, kind, mode), []).append(value)
        print(f'{mode}: matched repetition {index+1}/5', flush=True)
queries = {}
for index in range(5):
    for label in (('before', 'after') if index % 2 == 0 else ('after', 'before')):
        command = [str(getattr(args, label).resolve() / 'sl_bench_query')]
        commands.append(command)
        run = subprocess.run(command, capture_output=True, text=True, check=True, timeout=600)
        (args.output / f'{label}-query-{index+1}.json').write_text(run.stdout)
        queries.setdefault(label, []).append(report.decode(run.stdout))
summary = {'repetitions': 5, 'order': 'Alternating before/after by repetition; off then on.',
           'commands': commands, 'comparison': [], 'query': [],
           'timing_note': 'Whole-step or whole-query-batch time, same host; no speed acceptance threshold.'}
for mode in ('off', 'on'):
    for kind, deterministic in (('matrix', report.deterministic), ('study', study.deterministic)):
        reference = deterministic(data['before', kind, mode][0])
        for label in ('before', 'after'):
            assert all(deterministic(v) == reference for v in data[label, kind, mode]), (label, kind, mode)
        rows = []
        for i, scene in enumerate(data['before', kind, mode][0]['results']):
            row = {'scene': scene['scene']}
            for label in ('before', 'after'):
                times = [(v['results'][i]['timing_ms']['average'] if kind == 'matrix'
                          else v['results'][i]['average_ms']) for v in data[label, kind, mode]]
                row[label + '_average_ms'] = {'median': statistics.median(times), 'min': min(times), 'max': max(times)}
            rows.append(row)
        entry = {'sleep': mode, 'kind': kind, 'all_non_timing_results_equal': True, 'timing': rows}
        if kind == 'matrix':
            entry['metadata'] = {label: data[label,kind,mode][0]['metadata'] for label in ('before','after')}
            entry['deltas'] = report.compare(data['before',kind,mode][0],data['after',kind,mode][0],True)
        summary['comparison'].append(entry)
for i, row in enumerate(queries['before'][0]):
    entry = {'scene': row['scene'], 'digest': row['digest'], 'queries': row['queries']}
    for label in ('before', 'after'):
        assert all({k:v for k,v in run[i].items() if k != 'batch_ms'} ==
                   {k:v for k,v in row.items() if k != 'batch_ms'} for run in queries[label])
        times = [run[i]['batch_ms'] for run in queries[label]]
        entry[label + '_batch_ms'] = {'median':statistics.median(times),'min':min(times),'max':max(times)}
    summary['query'].append(entry)
(args.output / 'comparison.json').write_text(json.dumps(summary, indent=2)+'\n')
print('All matched results and quality gates passed.', flush=True)
