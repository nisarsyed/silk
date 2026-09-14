#!/usr/bin/env python3
"""Record the actual configured benchmark artifact; output stays in the build tree."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root', type=Path, required=True)
parser.add_argument('--build', type=Path, required=True)
parser.add_argument('--compiler', required=True)
parser.add_argument('--mode', required=True)
args = parser.parse_args()
def git(*command):
    return subprocess.check_output(['git', '-C', str(args.root), *command])
source = hashlib.sha256()
files = git('ls-files', '-z', '-c', '--others', '--exclude-standard', '--',
            'src', 'include', 'wasm', 'bench', 'cmake', 'CMakeLists.txt', 'CMakePresets.json')
for name in sorted(set(files.decode().split('\0')) - {''}):
    if name.startswith('bench/reports/'):
        continue
    path = args.root / name
    if path.is_file():
        source.update(name.encode() + b'\0' + path.read_bytes() + b'\0')
compiler = subprocess.check_output([args.compiler, '--version'], text=True).strip()
link = args.build / 'wasm/CMakeFiles/silk_wasm_bench.dir/link.txt'
if not link.is_file():
    raise SystemExit('Benchmark provenance needs the configured Makefile link.txt; use a WASM preset')
binary = args.build / 'benchmark/bench.wasm'
sdk_root = Path(os.environ.get('EMSDK', str(Path(args.compiler).resolve().parents[2])))
sdk_commit = subprocess.check_output(['git', '-C', str(sdk_root), 'rev-parse', 'HEAD'], text=True).strip()
value = {'revision': git('rev-parse', 'HEAD').decode().strip(),
         'dirty': bool(git('status', '--porcelain', '--untracked-files=normal')),
         'source_sha256': source.hexdigest(), 'wasm_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
         'compiler': compiler, 'mode': args.mode, 'link_flags': link.read_text().strip(),
         'compile_commands': json.loads((args.build / 'compile_commands.json').read_text()),
         'sdk_version': '6.0.9', 'sdk_commit': sdk_commit}
(args.build / 'benchmark/build.json').write_text(json.dumps(value, indent=2) + '\n')
