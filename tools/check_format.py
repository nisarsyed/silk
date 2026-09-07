#!/usr/bin/env python3
"""Check all Git-tracked first-party C sources/headers, never build dependencies."""
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    try:
        version = subprocess.check_output(['clang-format', '--version'], text=True)
        if 'version 22.1.3' not in version:
            raise ValueError('clang-format 22.1.3 is required')
        tracked = subprocess.check_output(['git', 'ls-files', '-z', '--', '*.c', '*.h'], cwd=root)
        files = [path.decode('utf-8') for path in tracked.split(b'\0') if path]
        if not files:
            raise ValueError('no tracked C sources/headers found')
        for offset in range(0, len(files), 100):
            subprocess.run(['clang-format', '--dry-run', '--Werror', *files[offset:offset+100]],
                           cwd=root, check=True)
        print(f'checked {len(files)} tracked C sources/headers')
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
