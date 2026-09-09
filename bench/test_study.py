#!/usr/bin/env python3
"""Check solver-study validation and event acceptance against a live public consumer."""
import argparse
import copy
from pathlib import Path
import subprocess
import sys
import unittest

import study

EXE = None


class StudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run([str(EXE)], capture_output=True, text=True, check=True, timeout=60)
        cls.value = study.validate(study.decode(result.stdout))

    def test_baseline_quality(self):
        self.assertEqual(study.violations(self.value), [])

    def test_replay_and_timing(self):
        result = subprocess.run([str(EXE)], capture_output=True, text=True, check=True, timeout=60)
        other = study.validate(study.decode(result.stdout))
        self.assertEqual(study.deterministic(self.value), study.deterministic(other))
        other['results'][0]['average_ms'] += 100
        self.assertEqual(study.deterministic(self.value), study.deterministic(other))

    def test_nonfinite_missing_and_wrong_fields(self):
        for field, bad in (('penetration_max', float('nan')), ('average_ms', float('inf')),
                           ('digest', 'bad'), ('prepared_contacts', True),
                           ('penetration_samples', [])):
            with self.subTest(field=field):
                value = copy.deepcopy(self.value)
                value['results'][0][field] = bad
                with self.assertRaises(ValueError):
                    study.validate(value)
        value = copy.deepcopy(self.value)
        del value['body_capacity']
        with self.assertRaises(ValueError):
            study.validate(value)
        value = copy.deepcopy(self.value)
        value['unknown'] = 0
        with self.assertRaises(ValueError):
            study.validate(value)

    def test_frozen_motion_fails(self):
        for scene in (1, 4):
            value = copy.deepcopy(self.value)
            value['results'][scene]['drop_max'] = 0
            self.assertTrue(study.violations(value))

    def test_load_and_momentum_fail(self):
        for scene, field in ((0, 'cached_support_force_mean'), (2, 'momentum_error_max')):
            value = copy.deepcopy(self.value)
            value['results'][scene][field] = 1e6
            self.assertTrue(study.violations(value))

    def test_invalid_cli(self):
        for args in (('--sleep',), ('--sleep', 'invalid'), ('--sleep', 'on', '--sleep', 'off'), ('--bad', 'off')):
            result = subprocess.run([str(EXE), *args], capture_output=True, timeout=30)
            self.assertNotEqual(result.returncode, 0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    args = parser.parse_args()
    EXE = args.executable.resolve()
    unittest.main(argv=[sys.argv[0]])
