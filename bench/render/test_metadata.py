#!/usr/bin/env python3
"""Reject stale/mismatched study artifacts before collecting browser evidence."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from build import module_record
from metadata import artifact, source_record


class ProvenanceTests(unittest.TestCase):
    def test_source_edits_deletions_and_untracked_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def git(*args):
                return subprocess.check_output(['git', '-C', directory, *args], stderr=subprocess.DEVNULL)
            git('init', '--quiet')
            (root/'src').mkdir()
            (root/'src/main.c').write_text('original\n')
            (root/'.gitignore').write_text('build/\n')
            git('add', '.')
            git('-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
                '-c', 'commit.gpgsign=false', 'commit', '--quiet', '-m', 'test')
            original = source_record(root)
            self.assertFalse(original['dirty'])
            (root/'build').mkdir()
            (root/'build/private.json').write_text('not source')
            self.assertEqual(source_record(root), original)
            (root/'src/main.c').write_text('edited\n')
            edited = source_record(root)
            self.assertTrue(edited['dirty'])
            self.assertNotEqual(original['sha256'], edited['sha256'])
            (root/'src/new.h').write_text('new\n')
            added = source_record(root)
            self.assertIn('src/new.h', added['files'])
            self.assertNotEqual(added['sha256'], edited['sha256'])
            (root/'src/main.c').unlink()
            deleted = source_record(root)
            self.assertIsNone(deleted['files']['src/main.c'])
            self.assertNotEqual(deleted['sha256'], added['sha256'])
            (root/'bench/reports').mkdir(parents=True)
            (root/'bench/reports/private.json').write_text('not build input')
            self.assertEqual(source_record(root)['sha256'], deleted['sha256'])

    def test_stale_or_modified_builds_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('study.mjs', 'study.wasm', 'driver.mjs', 'owner.mjs', 'views.mjs'):
                (root/name).write_bytes(name.encode())
            source = {'sha256': 'a'*64}
            value = {'schema': 1, 'kind': 'physics', 'source': source,
                     'artifacts': {p.name: artifact(p) for p in root.iterdir()}}
            (root/'build.json').write_text(json.dumps(value))
            self.assertEqual(module_record(root, 'physics', source), value)
            with self.assertRaisesRegex(ValueError, 'Stale physics'):
                module_record(root, 'physics', {'sha256': 'b'*64})
            with self.assertRaisesRegex(ValueError, 'Invalid raylib'):
                module_record(root, 'raylib', source)
            (root/'study.wasm').write_bytes(b'wrong binary')
            with self.assertRaisesRegex(ValueError, 'artifact differs'):
                module_record(root, 'physics', source)
            value['artifacts'] = {}
            (root/'build.json').write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, 'Incomplete'):
                module_record(root, 'physics', source)


if __name__ == '__main__':
    unittest.main()
