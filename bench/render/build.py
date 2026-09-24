#!/usr/bin/env python3
"""Assemble developer-only renderer correctness prototypes; no deployment."""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
from metadata import artifact, git, source_record

ROOT = Path(__file__).resolve().parents[2]


def module_record(directory, kind, source):
    value = json.loads((directory/'build.json').read_text())
    if value.get('schema') != 1 or value.get('kind') != kind:
        raise ValueError(f'Invalid {kind} build provenance')
    if value['source']['sha256'] != source['sha256']:
        raise ValueError(f'Stale {kind} source provenance; rebuild its CMake target before assembling')
    required = {'raylib.mjs', 'raylib.wasm'} if kind == 'raylib' else {
        'study.mjs', 'study.wasm', 'driver.mjs', 'owner.mjs', 'views.mjs'}
    if not required.issubset(value['artifacts']):
        raise ValueError(f'Incomplete {kind} artifact provenance')
    for name, expected in value['artifacts'].items():
        if Path(name).name != name or artifact(directory/name) != expected:
            raise ValueError(f'{kind} artifact differs from build provenance: {name}')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, default=ROOT/'build/wasm-release/benchmark')
    parser.add_argument('--physics', type=Path, help='Defaults to render-physics beside the benchmark directory')
    parser.add_argument('--raylib', type=Path, default=ROOT/'build/wasm-render/render-study')
    parser.add_argument('--output', type=Path, default=ROOT/'build/render-study')
    args = parser.parse_args()
    output = args.output.resolve()
    physics = args.physics or args.benchmark.parent/'render-physics'
    if output == ROOT/'build' or not output.is_relative_to(ROOT/'build'):
        parser.error('Output must be a child directory inside the ignored build directory')
    if any(directory.resolve().is_relative_to(output) for directory in (args.benchmark, args.raylib, physics)):
        parser.error('Output must not contain an input build directory')
    for source in (args.benchmark/'bench.wasm', args.raylib/'raylib.wasm', physics/'study.wasm'):
        if not source.is_file():
            parser.error(f'Build prerequisite missing: {source}')
    source = source_record(ROOT)
    try:
        modules = {kind: module_record(directory, kind, source)
                   for kind, directory in (('physics', physics), ('raylib', args.raylib))}
        if modules['physics']['mode'] != modules['raylib']['mode']:
            raise ValueError('Physics and raylib build modes must match')
        if any(modules['physics'][key] != modules['raylib'][key] for key in ('compiler', 'sdkCommit')):
            raise ValueError('Physics and raylib compiler/SDK provenance must match')
        benchmark = json.loads((args.benchmark/'build.json').read_text())
        if benchmark['wasm_sha256'] != artifact(args.benchmark/'bench.wasm')['sha256']:
            raise ValueError('Benchmark artifact differs from its build provenance')
        if benchmark['mode'] != modules['physics']['mode']:
            raise ValueError('Benchmark and study build modes must match')
    except (OSError, KeyError, ValueError) as error:
        parser.error(str(error))
    subprocess.run([str(ROOT/'wasm/node_modules/.bin/tsc'), '-p', str(ROOT/'bench/render/tsconfig.json'),
                    '--outDir', str(output)], check=True)
    for name in ('raylib.mjs', 'raylib.wasm'):
        shutil.copyfile(args.raylib/name, output/name)
    shutil.copytree(args.raylib/'licenses', output/'licenses', dirs_exist_ok=True)
    shutil.copytree(ROOT/'wasm/licenses', output/'licenses/emscripten', dirs_exist_ok=True)
    shutil.copyfile(ROOT/'LICENSE', output/'LICENSE')
    for name in ('verify.html', 'verify.mjs', 'runner.mjs', 'collect.html', 'collect.mjs'):
        shutil.copyfile(ROOT/'bench/render'/name, output/name)
    shutil.copytree(args.benchmark, output/'benchmark', dirs_exist_ok=True)
    shutil.copytree(physics, output/'physics', dirs_exist_ok=True)
    for kind, module in modules.items():
        directory = output/'physics' if kind == 'physics' else output
        for name, expected in module['artifacts'].items():
            if artifact(directory/name) != expected:
                raise RuntimeError(f'{kind} artifact changed during assembly: {name}')
    # Inventory the delivered files, including glue, copied wrappers, notices
    # and the original benchmark's separate build record. Exclude only these
    # generated manifest files to avoid a recursive checksum.
    value = {'schema': 1, 'verification': 'assembly-time', 'source': source,
             'contract': {'path': 'docs/browser-contract.md',
                          'declaredRevision': int(re.search(r'^Contract revision (\d+),',
                              (ROOT/'docs/browser-contract.md').read_text(), re.MULTILINE).group(1)),
                          'lastCommit': git(ROOT, 'log', '-1', '--format=%H', '--',
                                            'docs/browser-contract.md').decode().strip(),
                          'sha256': source['files']['docs/browser-contract.md']},
             'modules': modules,
             'typescript': subprocess.check_output([str(ROOT/'wasm/node_modules/.bin/tsc'), '--version'], text=True).strip(),
             'artifacts': {str(path.relative_to(output)): artifact(path) for path in sorted(output.rglob('*'))
                           if path.is_file() and path.name not in ('provenance.json', 'provenance.mjs')}}
    if source_record(ROOT) != source:
        raise RuntimeError('Sources changed during assembly; rerun the build')
    serialized = json.dumps(value, sort_keys=True, separators=(',', ':'))
    (output/'provenance.json').write_text(serialized+'\n')
    (output/'provenance.mjs').write_text('export const provenanceJson='+json.dumps(serialized)+';\n')
    print(output)

if __name__ == '__main__':
    main()
