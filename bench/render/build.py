#!/usr/bin/env python3
"""Assemble developer-only renderer correctness prototypes; no deployment."""
import argparse
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, default=ROOT/'build/wasm-release/benchmark')
    parser.add_argument('--physics', type=Path, help='Defaults to render-physics beside the benchmark directory')
    parser.add_argument('--raylib', type=Path, default=ROOT/'build/wasm-render/render-study')
    parser.add_argument('--output', type=Path, default=ROOT/'build/render-study')
    args = parser.parse_args()
    output = args.output.resolve()
    physics = args.physics or args.benchmark.parent/'render-physics'
    if not output.is_relative_to(ROOT/'build'):
        parser.error('Output must be inside the ignored build directory')
    for source in (args.benchmark/'bench.wasm', args.raylib/'raylib.wasm', physics/'study.wasm'):
        if not source.is_file():
            parser.error(f'Build prerequisite missing: {source}')
    subprocess.run([str(ROOT/'wasm/node_modules/.bin/tsc'), '-p', str(ROOT/'bench/render/tsconfig.json'),
                    '--outDir', str(output)], check=True)
    for name in ('raylib.mjs', 'raylib.wasm'):
        shutil.copyfile(args.raylib/name, output/name)
    shutil.copytree(args.raylib/'licenses', output/'licenses', dirs_exist_ok=True)
    shutil.copytree(ROOT/'wasm/licenses', output/'licenses/emscripten', dirs_exist_ok=True)
    shutil.copyfile(ROOT/'LICENSE', output/'LICENSE')
    for name in ('verify.html', 'verify.mjs'):
        shutil.copyfile(ROOT/'bench/render'/name, output/name)
    shutil.copytree(args.benchmark, output/'benchmark', dirs_exist_ok=True)
    shutil.copytree(physics, output/'physics', dirs_exist_ok=True)
    print(output)

if __name__ == '__main__':
    main()
