#!/usr/bin/env python3
"""Run parser contracts against a freshly generated short WASM report."""
import argparse
import copy
import json
from pathlib import Path
import tempfile
import unittest
import validate

EXAMPLE = None
class Reports(unittest.TestCase):
    def setUp(self):
        self.report = copy.deepcopy(EXAMPLE)
    def test_live_structure_and_same_build(self):
        validate.validate(self.report)
        validate.compare(self.report, copy.deepcopy(self.report))
    def test_smoke_cannot_claim_settling(self):
        with self.assertRaises(ValueError):
            validate.validate(self.report, quality=True)
        self.report['profile']['settling_profile'] = True
        with self.assertRaises(ValueError):
            validate.validate(self.report)
    def test_exact_counters(self):
        self.assertEqual(validate.decimal('9007199254740993'), 9007199254740993)
        self.assertEqual(validate.decimal('18446744073709551615'), 2**64-1)
        for value in (9007199254740993, True, '01', '-1', '1.0', '1e3', '18446744073709551616'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate.decimal(value)
        self.report['native']['results'][0]['cumulative_work']['tree_node_visits'] = '9007199254740993'
        result = validate.validate(self.report)
        self.assertEqual(result['results'][0]['cumulative_work']['tree_node_visits'], 9007199254740993)
    def test_regions_and_memory_cannot_lie(self):
        for group, key, bad in (
            ('profile','measured_steps',1), ('runtime','visibility_end','hidden'),
            ('build','sdk_version','wrong'), ('module','memory_bytes',2097153)):
            value = copy.deepcopy(self.report); value[group][key] = bad
            with self.subTest(key=key), self.assertRaises(ValueError): validate.validate(value)
        for key, bad in (('step_samples_ms',[]), ('step_ms',1e10), ('quality_ms',float('nan'))):
            value = copy.deepcopy(self.report); value['regions'][0][key] = bad
            with self.subTest(key=key), self.assertRaises(ValueError): validate.validate(value)
        for key in ('requested_bytes','sample_bytes','output_bytes','adapter_bytes'):
            value = copy.deepcopy(self.report); value['regions'][0]['memory'][key] += 1
            with self.subTest(key=key), self.assertRaises(ValueError): validate.validate(value)
    def test_both_linker_stack_layouts(self):
        memory = self.report['regions'][0]['memory']
        # Keep the occupied/requested payload fixed; only change linker layout.
        memory['static_end'] = 4568
        memory['heap_base'] = 1053152
        validate.validate(self.report)
        memory['heap_base'] += 4
        with self.assertRaises(ValueError): validate.validate(self.report)
    def test_failed_and_partial_runs_never_pass(self):
        for key, value in (('execution_status','failed'), ('failures',[{'stage':'step'}]), ('regions',[])):
            report = copy.deepcopy(self.report); report[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError): validate.validate(report)
    def test_report_and_provenance_fields(self):
        for key in self.report:
            value = copy.deepcopy(self.report); del value[key]
            with self.subTest(key=key), self.assertRaises(ValueError): validate.validate(value)
        self.report['unexpected'] = True
        with self.assertRaises(ValueError): validate.validate(self.report)
    def test_malformed_and_bounded_json(self):
        with tempfile.TemporaryDirectory() as root:
            file = Path(root)/'report.json'
            for raw in ('{"a":1,"a":2}', '{"value":NaN}', '{', ' '* (validate.MAX_BYTES+1)):
                file.write_text(raw)
                with self.assertRaises((ValueError,json.JSONDecodeError)): validate.load(file)
    def test_same_build_requires_matching_code_and_results(self):
        other = copy.deepcopy(self.report); other['build']['wasm_sha256'] = '0'*64
        with self.assertRaises(ValueError): validate.compare(self.report,other)
        other = copy.deepcopy(self.report); other['native']['results'][0]['semantic_digest'] = '0'*16
        with self.assertRaises(ValueError): validate.compare(self.report,other)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--report',required=True)
    args = parser.parse_args(); EXAMPLE = validate.load(args.report)
    unittest.main(argv=[__file__])
