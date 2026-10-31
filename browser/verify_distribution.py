#!/usr/bin/env python3
"""Verify a downloaded Pages artifact without rebuilding any of its inputs."""
import hashlib
import json
from pathlib import Path
import re
import sys


def fail(reason):
    raise SystemExit(reason)


def main():
    if len(sys.argv) != 3:
        fail('Usage: verify_distribution.py SITE EXPECTED_SOURCE_SHA')
    site = Path(sys.argv[1]).resolve()
    revision = sys.argv[2]
    if not re.fullmatch('[0-9a-f]{40}', revision):
        fail('Expected source revision must be a full commit SHA')
    record = json.loads((site/'distribution.json').read_text())
    if record.get('schema') != 1 or record.get('source', {}).get('revision') != revision:
        fail('Artifact provenance does not match the selected verified run')
    listed = record.get('artifacts')
    if not isinstance(listed, dict):
        fail('Artifact inventory is missing')
    actual = set()
    for path in site.rglob('*'):
        if path.is_symlink():
            fail(f'Site contains a symbolic link: {path}')
        if path.is_file():
            actual.add(str(path.relative_to(site)))
    if actual != set(listed) | {'distribution.json', 'checksums.sha256'}:
        fail('Site files differ from the distribution inventory')
    checks = {}
    for line in (site/'checksums.sha256').read_text().splitlines():
        found = re.fullmatch(r'([0-9a-f]{64})  (.+)', line)
        if not found or found.group(2) in checks:
            fail('Invalid or duplicate checksum line')
        checks[found.group(2)] = found.group(1)
    if set(checks) != set(listed) | {'distribution.json'}:
        fail('Checksum inventory differs from the site')
    for name in checks:
        path = site/name
        if path.resolve() != path or not path.is_file() or not path.is_relative_to(site):
            fail(f'Invalid artifact path: {name}')
        content = path.read_bytes()
        sha = hashlib.sha256(content).hexdigest()
        if sha != checks[name]:
            fail(f'Checksum mismatch: {name}')
        if name in listed and listed[name] != {'bytes': len(content), 'sha256': sha}:
            fail(f'Distribution metadata mismatch: {name}')
    archive = record.get('package', {}).get('archive')
    if archive not in listed or record['package']['sha256'] != checks[archive]:
        fail('Download checksum differs from the distribution metadata')
    print(f'Verified browser artifact for {revision}')


if __name__ == '__main__':
    main()
