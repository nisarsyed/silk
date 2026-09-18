#!/usr/bin/env python3
"""Stamp built study modules; generated provenance stays in the build tree."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


def git(root, *command):
    return subprocess.check_output(['git', '-C', str(root), *command])


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sources(root):
    # Include local untracked source edits, deleted tracked files, JS wrappers,
    # build scripts and the frozen contract. Ignore generated/private reports.
    names = git(root, 'ls-files', '-z', '-c', '--others', '--exclude-standard', '--',
                'src', 'include', 'wasm', 'bench', 'cmake', 'CMakeLists.txt',
                'CMakePresets.json', 'docs/browser-contract.md')
    result = {}
    for name in sorted(set(names.decode().split('\0')) - {''}):
        if name.startswith('bench/reports/'):
            continue
        path = root / name
        result[name] = digest(path) if path.is_file() else None
    return result


def source_record(root):
    files = sources(root)
    encoded = json.dumps(files, sort_keys=True, separators=(',', ':')).encode()
    return {'revision': git(root, 'rev-parse', 'HEAD').decode().strip(),
            'dirty': bool(git(root, 'status', '--porcelain', '--untracked-files=normal')),
            'sha256': hashlib.sha256(encoded).hexdigest(), 'files': files}


def artifact(path):
    return {'bytes': path.stat().st_size, 'sha256': digest(path)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--compiler', required=True)
    parser.add_argument('--mode', required=True)
    parser.add_argument('--kind', choices=('physics', 'raylib'), required=True)
    parser.add_argument('--raylib-source', type=Path)
    args = parser.parse_args()
    root, build = args.root.resolve(), args.build.resolve()
    target, relative = (('silk_wasm_study', 'wasm') if args.kind == 'physics'
                        else ('silk_render_raylib', 'bench/render'))
    output = build / ('render-physics' if args.kind == 'physics' else 'render-study')
    link = build / relative / 'CMakeFiles' / (target + '.dir') / 'link.txt'
    if not link.is_file():
        parser.error('Study provenance requires the configured Makefile link.txt')
    commands = json.loads((build / 'compile_commands.json').read_text())
    targets = [target, 'silk'] + (['raylib'] if args.kind == 'raylib' else [])
    commands = [row for row in commands if any('CMakeFiles/' + name + '.dir/' in
                row.get('command', ' '.join(row.get('arguments', []))) for name in targets)]
    for name in targets:
        if not any('CMakeFiles/' + name + '.dir/' in row.get('command',
                   ' '.join(row.get('arguments', []))) for row in commands):
            parser.error(f'Missing {name} compilation commands; configure CMAKE_EXPORT_COMPILE_COMMANDS=ON')
    sdk = Path(os.environ.get('EMSDK', str(Path(args.compiler).resolve().parents[2])))
    value = {'schema': 1, 'kind': args.kind, 'source': source_record(root),
             'compiler': subprocess.check_output([args.compiler, '--version'], text=True).strip(),
             'sdkCommit': git(sdk, 'rev-parse', 'HEAD').decode().strip(),
             'mode': args.mode, 'linkCommand': link.read_text().strip(), 'compileCommands': commands,
             'artifacts': {path.name: artifact(path) for path in sorted(output.iterdir())
                           if path.suffix in ('.mjs', '.wasm')}}
    if args.kind == 'raylib':
        if not args.raylib_source:
            parser.error('Raylib source directory is required')
        # Fingerprint actual vendored source, including edits to an overridden
        # FetchContent checkout; the requested tag alone is not provenance.
        raylib = args.raylib_source.resolve()
        value['raylib'] = {'requestedTag': '6.0', 'files': {
            str(path.relative_to(raylib)): digest(path) for path in sorted((raylib / 'src').rglob('*'))
            if path.is_file() and '.git' not in path.parts}}
    (output / 'build.json').write_text(json.dumps(value, indent=2) + '\n')


if __name__ == '__main__':
    main()
