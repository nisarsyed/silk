#!/usr/bin/env python3
"""Build external source and relocated-package consumers without dependencies."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', choices=('Debug', 'Release'), default='Debug')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output.resolve()
    work = Path(tempfile.mkdtemp(prefix='work-', dir=output))
    source, project = work / 'source', work / 'consumer'
    records = []
    summary = {'config': args.config, 'commands': records, 'passed': False}
    try:
        summary['revision'] = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
        summary['dirty'] = bool(subprocess.check_output(
            ['git', 'status', '--porcelain'], cwd=root, text=True).strip())
        source.mkdir()
        shutil.copy2(root / 'CMakeLists.txt', source)
        for name in ('src', 'include', 'cmake'):
            shutil.copytree(root / name, source / name)
        shutil.copytree(root / 'tests/consumer', project)
        shutil.copy2(root / 'examples/headless/main.c', project / 'main.c')
        with (output / 'checks.log').open('w') as log:
            def run(command, expected=0):
                command = [str(part) for part in command]
                log.write('\n' + repr(command) + '\n')
                log.flush()
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                        timeout=300, cwd=work)
                records.append({'command': command, 'exit_code': result.returncode})
                if (expected == 0 and result.returncode != 0) or (
                        expected != 0 and result.returncode == 0):
                    raise ValueError(f'unexpected exit {result.returncode}: {command}')

            def configure(directory, source_dir, *options, expected=0):
                run(['cmake', '-S', source_dir, '-B', directory,
                     '-DCMAKE_BUILD_TYPE=' + args.config,
                     '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF',
                     '-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF', *options], expected)

            def build_consumer(directory):
                run(['cmake', '--build', directory, '--config', args.config, '-j', '2'])
                options = (directory / f'consumer-options-{args.config}.txt').read_text()
                if options.strip():
                    raise ValueError(f'Silk leaked compile options into its consumer: {options}')
                suffix = '.exe' if os.name == 'nt' else ''
                for name in ('consumer', 'linkage'):
                    binary = directory / (name + suffix)
                    if not binary.exists():
                        binary = directory / args.config / (name + suffix)
                    run([binary])

            configure(work / 'embedded', project, '-DSILK_SOURCE=' + str(source),
                      '-DSILK_PUBLIC_HEADERS=' + str(source / 'include'))
            build_consumer(work / 'embedded')
            prefix = work / 'prefix'
            # A non-default include directory catches hard-coded header paths.
            # Keep lib in CMake's standard prefix search layout.
            configure(work / 'library', source, '-DSL_BUILD_TESTS=OFF',
                      '-DCMAKE_INSTALL_PREFIX=' + str(prefix),
                      '-DCMAKE_INSTALL_LIBDIR=lib',
                      '-DCMAKE_INSTALL_INCLUDEDIR=include/silk-sdk')
            run(['cmake', '--build', work / 'library', '--config', args.config, '-j', '2'])
            run(['cmake', '--install', work / 'library', '--config', args.config])
            installed = sorted(p.relative_to(prefix).as_posix() for p in prefix.rglob('*') if p.is_file())
            headers = sorted(p.name for p in (source / 'include/silk').glob('*.h'))
            actual_headers = sorted(p.name for p in (prefix / 'include/silk-sdk/silk').glob('*.h'))
            if actual_headers != headers:
                raise ValueError('installed public header inventory mismatch')
            allowed = {'include/silk-sdk/silk/' + name for name in headers}
            allowed.update({'lib/libsilk.a', 'lib/silk.lib',
                            'lib/cmake/silk/silkConfig.cmake',
                            'lib/cmake/silk/silkConfigVersion.cmake',
                            'lib/cmake/silk/silkTargets.cmake',
                            'lib/cmake/silk/silkTargets-' + args.config.lower() + '.cmake'})
            if any(name not in allowed for name in installed):
                raise ValueError(f'unexpected installed files: {installed}')
            for path in prefix.rglob('*.cmake'):
                content = path.read_text()
                if str(source) in content or str(work / 'library') in content or 'silk_warnings' in content:
                    raise ValueError(f'non-relocatable or private export: {path}')
            relocated = work / 'relocated'
            prefix.rename(relocated)
            source.rename(work / 'source-hidden')
            (work / 'library').rename(work / 'library-hidden')
            # The original source, build and prefix paths no longer exist.
            # Consumers now have only copied app sources and installed headers.
            options = ['-DCMAKE_PREFIX_PATH=' + str(relocated),
                       '-DSILK_PUBLIC_HEADERS=' + str(relocated / 'include/silk-sdk')]
            configure(work / 'installed', project, *options, '-DSILK_REQUESTED_VERSION=0.4.0')
            build_consumer(work / 'installed')
            for version in ('0.3.0', '0.4.1', '0.5.0'):
                configure(work / ('reject-' + version), project, *options,
                          '-DSILK_REQUESTED_VERSION=' + version, expected=1)
            summary.update({'passed': True, 'headers': headers, 'installed_files': installed,
                            'relocated_source_unavailable': True, 'downstream_compile_options': []})
        print('external source and relocated-package consumers passed')
        return 0
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        summary['error'] = str(error)
        print(f'consumer check: {error}; see {output / "checks.log"}', file=sys.stderr)
        return 1
    finally:
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    sys.exit(main())
