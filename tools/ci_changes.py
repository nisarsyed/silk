#!/usr/bin/env python3
"""Decide which CI jobs run, from the changed files and the CI tier.

Two tiers:

- quick (pull requests): the checks that catch most breakage fast.
- full (pushes to main, manual runs, pull requests labelled `full-ci`, and
  any change to CI itself): every platform, compiler, and build type.

Files on the documentation allowlist never trigger builds. Anything else does,
so a new kind of file is built and tested by default. When the change set
cannot be computed, everything runs.

Usage:
    ci_changes.py --base SHA --head SHA [--full]   (writes $GITHUB_OUTPUT)
    ci_changes.py --all
    ci_changes.py --self-test
"""
import argparse
from fnmatch import fnmatch
import json
import os
import subprocess
import sys

# Paths that cannot affect a build or a test result.
DOCS = (
    '*.md',
    'docs/*',
    'LICENSE',
    'LICENSE.*',
    '.gitignore',
    '.github/ISSUE_TEMPLATE/*',
    '.github/PULL_REQUEST_TEMPLATE.md',
)

# Paths that can change how the sandbox compiles: its source, the public
# headers it includes, and the build configuration.
SANDBOX = (
        'examples/sandbox/*',
    'examples/pool/*',
    'include/*',
    'CMakeLists.txt',
    'CMakePresets.json',
    'cmake/*',
    'src/CMakeLists.txt',
)

# Changes to CI itself run the full tier and the sandbox builds, so every job
# of the new configuration runs before it guards main.
CI = (
    '.github/workflows/*',
    'tools/ci_changes.py',
)


def matches(path, patterns):
    return any(fnmatch(path, pattern) for pattern in patterns)


def plan(files, full):
    """Job selection for a list of changed paths (None: unknown, run all)."""
    if files is None:
        code, sandbox, full = True, True, True
    else:
        code = any(not matches(path, DOCS) for path in files)
        ci = any(matches(path, CI) for path in files)
        sandbox = ci or any(matches(path, SANDBOX) for path in files)
        full = full or ci
    if not code:
        sandbox = False
    return {
        'code': code,
        'full': code and full,
        'linux_debug': ['gcc', 'clang'] if full else ['gcc'],
        'linux_release': ['gcc', 'clang'] if full else ['clang'],
        'sandbox': (['sandbox', 'sandbox-release'] if full
                    else ['sandbox-release']) if sandbox else [],
    }


def changed_files(base, head):
    if not base or set(base) == {'0'}:
        return None
    try:
        out = subprocess.check_output(
            ['git', 'diff', '--name-only', f'{base}...{head}'], text=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    return [line for line in out.splitlines() if line]


def write(selection):
    lines = [f'{key}={json.dumps(value)}' for key, value in selection.items()]
    target = os.environ.get('GITHUB_OUTPUT')
    if target:
        with open(target, 'a', encoding='utf-8') as out:
            out.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


def self_test():
    cases = [
        (['README.md', 'docs/solver.md', 'ROADMAP.md'], False,
         dict(code=False, full=False, sandbox=[])),
        (['src/solver.c'], False,
         dict(code=True, full=False, linux_debug=['gcc'], sandbox=[])),
        (['include/silk/world.h'], False,
         dict(code=True, sandbox=['sandbox-release'])),
        (['examples/sandbox/main.c'], True,
         dict(full=True, sandbox=['sandbox', 'sandbox-release'])),
        (['.github/workflows/ci.yml'], False,
         dict(code=True, full=True, linux_release=['gcc', 'clang'],
              sandbox=['sandbox', 'sandbox-release'])),
        (['tools/check_format.py'], False, dict(code=True, full=False)),
        (['bench/new_kind_of_file.xyz'], False, dict(code=True)),
        (['docs/solver.md'], True, dict(code=False, full=False)),
        (None, False, dict(code=True, full=True,
                           sandbox=['sandbox', 'sandbox-release'])),
    ]
    failures = 0
    for files, full, expected in cases:
        got = plan(files, full)
        for key, value in expected.items():
            if got[key] != value:
                print(f'FAIL {files} full={full}: {key}={got[key]!r}, '
                      f'expected {value!r}', file=sys.stderr)
                failures += 1
    print(f'{len(cases)} cases, {failures} failures')
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--base')
    parser.add_argument('--head', default='HEAD')
    parser.add_argument('--full', action='store_true')
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    files = None if args.all else changed_files(args.base, args.head)
    write(plan(files, args.full))
    return 0


if __name__ == '__main__':
    sys.exit(main())
