#!/usr/bin/env python3
"""Create a checked static demo and installable package from one WASM build."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'bench/render'))
from metadata import artifact, source_record  # noqa: E402

STATIC_FILES = (
    'sandbox.html', 'sandbox.css', 'sandbox.mjs',
    'runtime_controller.mjs', 'runtime_owner.mjs', 'runtime_worker.mjs',
    'pointer_adapter.mjs', 'geometry.js', 'canvas.js', 'webgl.js',
    'overlay.js', 'overlay_webgl.js', 'host_clock.mjs',
    'host_input_queue.mjs', 'host_input_trace.mjs',
    'host_frame_trace.mjs', 'host_presentation_clock.mjs',
    'provenance.mjs', 'provenance.json', 'LICENSE',
    'physics/driver.mjs', 'physics/owner.mjs', 'physics/views.mjs',
    'physics/study.mjs', 'physics/study.wasm',
)


def source_version():
    found = re.search(r'^project\(silk VERSION (\d+\.\d+\.\d+)',
                      (ROOT/'CMakeLists.txt').read_text(), re.MULTILINE)
    if not found:
        raise ValueError('Cannot read the engine version')
    return found.group(1)


def add_download_panel(html, filename, version, revision):
    marker = '</main>'
    if html.count(marker) != 1:
        raise ValueError('Sandbox document has no unique distribution slot')
    panel = (
        '<section class="download-panel" aria-labelledby="download-title">'
        '<div><p class="eyebrow">02 / DOWNLOAD</p>'
        '<h2 id="download-title">Take the engine with you.</h2>'
        f'<p>Silk {version} · build {revision[:7]} · portable WebAssembly package</p></div>'
        f'<a class="download-button" href="./downloads/{filename}" download>'
        'Download npm archive</a>'
        '<a class="checksum-link" href="./checksums.sha256" download>'
        'SHA-256 checksums</a></section>'
    )
    return html.replace(marker, marker + '\n' + panel, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assembled', type=Path,
                        default=ROOT/'build/render-study-release')
    parser.add_argument('--package', type=Path,
                        default=ROOT/'build/wasm-release/package')
    parser.add_argument('--output', type=Path,
                        default=ROOT/'build/browser-site')
    args = parser.parse_args()
    assembled, package, output = (value.resolve()
                                  for value in (args.assembled, args.package,
                                                args.output))
    if output == ROOT/'build' or not output.is_relative_to(ROOT/'build'):
        parser.error('Output must be a child of the ignored build directory')
    if assembled.is_relative_to(output) or package.is_relative_to(output):
        parser.error('Output cannot contain an input build')
    try:
        provenance = json.loads((assembled/'provenance.json').read_text())
        benchmark = json.loads((assembled/'benchmark/build.json').read_text())
        package_info = json.loads((package/'package.json').read_text())
        package_build = json.loads((package/'build.json').read_text())
        source = source_record(ROOT)
        if source['dirty'] or provenance['source'] != source:
            raise ValueError('Sources changed after the WASM build; build a clean commit')
        if benchmark['source_sha256'] != source['sha256']:
            raise ValueError('Package and browser builds have different sources')
        if (provenance['modules']['physics']['mode'] != 'Release' or
                provenance['modules']['raylib']['mode'] != 'Release' or
                benchmark['mode'] != 'Release' or
                package_build['mode'] != 'Release'):
            raise ValueError('Distribution requires matched release builds')
        if package_build['kind'] != 'package' or package_build['source'] != source:
            raise ValueError('Package build provenance differs from this source')
        if any(package_build[key] != provenance['modules']['physics'][key]
               for key in ('compiler', 'sdkCommit')):
            raise ValueError('Package toolchain differs from the browser build')
        for name, expected in package_build['artifacts'].items():
            if artifact(package/name) != expected:
                raise ValueError(f'Package asset differs from build provenance: {name}')
        version = source_version()
        if package_info['version'] != version or package_info['name'] != '@nisarsyed/silk':
            raise ValueError('Package version/name differs from this source')
        for name in STATIC_FILES:
            path = assembled/name
            if not path.is_file():
                raise ValueError(f'Missing browser asset: {name}')
            expected = provenance['artifacts'].get(name)
            if expected is not None and artifact(path) != expected:
                raise ValueError(f'Browser asset differs from provenance: {name}')
        if not (assembled/'licenses/emscripten').is_dir():
            raise ValueError('Missing Emscripten attribution')
        with tempfile.TemporaryDirectory(prefix='silk-site-',
                                         dir=ROOT/'build') as temporary:
            site = Path(temporary)
            for name in STATIC_FILES:
                target = site/name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(assembled/name, target)
            shutil.copytree(assembled/'licenses/emscripten',
                            site/'licenses/emscripten')
            downloads = site/'downloads'
            downloads.mkdir()
            result = subprocess.run(
                ['npm', 'pack', '--json', '--pack-destination', str(downloads)],
                cwd=package, check=True, capture_output=True, text=True)
            packed = json.loads(result.stdout)
            if len(packed) != 1:
                raise ValueError('npm pack emitted an unexpected archive list')
            package_files = {entry['path'] for entry in packed[0]['files']}
            required_files = {'package.json', 'build.json', 'index.mjs',
                              'index.d.ts', 'views.mjs', 'silk.mjs', 'silk.wasm',
                              'README.md', 'LICENSE', 'licenses/emscripten.txt',
                              'licenses/musl.txt', 'licenses/compiler-rt.txt',
                              'licenses/fmaf.txt'}
            if not required_files.issubset(package_files):
                raise ValueError('npm archive omits package code, metadata, or attribution')
            filename = packed[0]['filename']
            if Path(filename).name != filename or not (downloads/filename).is_file():
                raise ValueError('npm pack did not produce one local archive')
            html = add_download_panel((site/'sandbox.html').read_text(),
                                      filename, version, source['revision'])
            for name in ('index.html', 'sandbox.html'):
                (site/name).write_text(html)
            with (site/'sandbox.css').open('a') as css:
                css.write('\n.download-panel{max-width:1850px;margin:0 auto 35px;'
                          'padding:20px clamp(20px,3vw,52px);border-top:1px solid var(--line);'
                          'display:flex;align-items:center;gap:18px}'
                          '.download-panel>div{flex:1}.download-panel .eyebrow{margin:0 0 5px}'
                          '.download-panel h2{font:400 29px Georgia,serif;margin:0}'
                          '.download-panel p:last-child{color:var(--muted);margin:4px 0 0}'
                          '.download-button{padding:12px 16px;background:var(--ink);'
                          'color:var(--paper);text-decoration:none;font:600 11px Menlo,monospace}'
                          '.checksum-link{color:var(--ink);font:11px Menlo,monospace}'
                          '@media(max-width:780px){.download-panel{display:grid;gap:12px;'
                          'margin-bottom:10px}.download-button,.checksum-link{justify-self:start}}\n')
            files = {str(path.relative_to(site)): artifact(path)
                     for path in sorted(site.rglob('*')) if path.is_file()}
            record = {
                'schema': 1, 'version': version,
                'source': {'revision': source['revision'],
                           'sha256': source['sha256']},
                'toolchain': {'compiler': provenance['modules']['physics']['compiler'],
                              'sdkCommit': provenance['modules']['physics']['sdkCommit'],
                              'typescript': provenance['typescript']},
                'assemblySha256': hashlib.sha256(
                    (assembled/'provenance.json').read_bytes()).hexdigest(),
                'package': {'name': package_info['name'], 'version': version,
                            'archive': f'downloads/{filename}',
                            **files[f'downloads/{filename}']},
                'artifacts': files,
            }
            (site/'distribution.json').write_text(
                json.dumps(record, sort_keys=True, indent=2) + '\n')
            checks = {**files, 'distribution.json': artifact(site/'distribution.json')}
            (site/'checksums.sha256').write_text(''.join(
                f"{item['sha256']}  {name}\n" for name, item in sorted(checks.items())))
            if output.exists():
                shutil.rmtree(output)
            shutil.copytree(site, output)
        print(output)
    except (OSError, KeyError, ValueError, subprocess.CalledProcessError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
