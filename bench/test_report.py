#!/usr/bin/env python3
"""Developer-tool contract checks; no timing thresholds and no CTest entry."""
import argparse
import copy
import math
from pathlib import Path
import subprocess
import sys
import unittest

sys.dont_write_bytecode = True
import report

EXE = None
BASE = Path(__file__).parent / 'reports' / 'matrix' / 'run-1.json'


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.baseline = report.load(BASE)

    def test_valid_and_quality(self):
        report.validate(self.baseline)
        report.quality_check(self.baseline)

    def test_schema_version(self):
        for version in (1, 3, True, '2'):
            value = copy.deepcopy(self.baseline)
            value['schema_version'] = version
            with self.assertRaises(ValueError):
                report.validate(value)

    def test_malformed_json(self):
        for text in ('{', '{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}', '[' * 2000):
            with self.subTest(text=text[:40]):
                with self.assertRaises((ValueError, RecursionError)):
                    report.decode(text)

    def test_report_size_bound(self):
        with self.assertRaises(ValueError):
            report.decode(' ' * 1_048_577)

    def test_missing_and_unknown_fields(self):
        for path in ((), ('metadata',), ('results', 0), ('results', 0, 'settings'),
                     ('results', 0, 'memory_bytes'), ('results', 0, 'quality')):
            for extra in (False, True):
                value = copy.deepcopy(self.baseline)
                target = value
                for part in path:
                    target = target[part]
                if extra:
                    target['unexpected'] = 0
                else:
                    del target[next(iter(target))]
                with self.subTest(path=path, extra=extra):
                    with self.assertRaises(ValueError):
                        report.validate(value)

    def test_nonfinite_and_wrong_numeric_types(self):
        for bad in (math.nan, math.inf, -math.inf, '0', True, None, []):
            value = copy.deepcopy(self.baseline)
            value['results'][0]['timing_ms']['average'] = bad
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    report.validate(value)

    def test_semantic_invariants(self):
        changes = [
            (('drops',), 1), (('cumulative_work', 'contact_drops'), 1),
            (('counts', 'bodies'), 65537), (('counts', 'pairs'), 0),
            (('settings', 'measured_steps'), 0), (('settings', 'substeps'), 9),
            (('settings', 'restitution'), 2), (('settings', 'sleep_enabled'), True),
            (('memory_bytes', 'arena'), 1), (('step', 'substeps'), 0),
            (('timing_ms', 'p95'), 1e8), (('quality', 'window_steps'), 0),
            (('semantic_digest',), 'not-a-digest'), (('step', 'work', 'pair_probes'), 2**64),
        ]
        for path, bad in changes:
            value = copy.deepcopy(self.baseline)
            target = value['results'][0]
            for part in path[:-1]:
                target = target[part]
            target[path[-1]] = bad
            with self.subTest(path=path):
                with self.assertRaises(ValueError):
                    report.validate(value)

    def test_deterministic_mismatch(self):
        changed = copy.deepcopy(self.baseline)
        changed['results'][0]['semantic_digest'] = '0' * 16
        with self.assertRaisesRegex(ValueError, 'replay mismatch'):
            report.compare(self.baseline, changed)

    def test_timing_and_metadata_do_not_change_replay(self):
        changed = copy.deepcopy(self.baseline)
        changed['metadata']['host'] = 'different host label'
        for row in changed['results']:
            row['timing_ms'] = {key: value * 2 for key, value in row['timing_ms'].items()}
        report.compare(self.baseline, changed)

    def test_cross_build_requires_matched_settings_and_quality(self):
        changed = copy.deepcopy(self.baseline)
        changed['results'][0]['semantic_digest'] = '0' * 16
        report.compare(self.baseline, changed, cross_build=True)
        changed['results'][0]['quality']['penetration_max'] = 10
        with self.assertRaises(ValueError):
            report.compare(self.baseline, changed, cross_build=True)
        changed = copy.deepcopy(self.baseline)
        changed['results'][0]['settings']['warmup_steps'] = 121
        with self.assertRaises(ValueError):
            report.compare(self.baseline, changed, cross_build=True)

    def test_short_profile_is_not_settled_quality_evidence(self):
        changed = copy.deepcopy(self.baseline)
        changed['results'][0]['settings']['warmup_steps'] = 2
        with self.assertRaises(ValueError):
            report.quality_check(changed)

    def test_cli_rejection(self):
        if EXE is None:
            self.skipTest('pass --executable for live CLI checks')
        cases = [('--steps', '0'), ('--steps', '10001'), ('--steps', '-1'),
                 ('--steps', '99999999999999999'), ('--steps', '1x'), ('--warmup',),
                 ('--scene', 'bogus'), ('--scene', 'baseline'),
                 ('--format', 'json'), ('--format', 'text'), ('--unknown', '1'),
                 ('--steps', '1', '--steps', '2'), ('--warmup', '+2')]
        for args in cases:
            with self.subTest(args=args):
                result = subprocess.run([str(EXE), *args], capture_output=True, timeout=30)
                self.assertNotEqual(result.returncode, 0)

    def test_minimum_profile(self):
        if EXE is None:
            self.skipTest('pass --executable for live CLI checks')
        result = subprocess.run([str(EXE), '--scene', 'table', '--warmup', '0', '--steps', '1'], capture_output=True, text=True, timeout=30, check=True)
        report.validate(report.decode(result.stdout))

    def test_default_scene_selection(self):
        if EXE is None:
            self.skipTest('pass --executable for live CLI checks')
        result = subprocess.run([str(EXE), '--warmup', '0', '--steps', '1'],
                                capture_output=True, text=True, timeout=30, check=True)
        value = report.decode(result.stdout)
        report.validate(value)
        self.assertEqual({row['scene'] for row in value['results']},
                         {'pyramid', 'rain', 'piles', 'chains', 'churn', 'table', 'inverted'})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path)
    args = parser.parse_args()
    EXE = args.executable.resolve() if args.executable else None
    unittest.main(argv=[sys.argv[0]])
