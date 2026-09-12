# WebAssembly toolchain

The opt-in `wasm-debug` and `wasm-release` presets compile the portable core
and complete C test runner for scalar wasm32. They do not build a browser UI.
Native builds and installed C consumers need neither Emscripten nor Node.
The [browser contract](browser-contract.md) defines the remaining package and
browser acceptance work.

## Setup

Install the pinned SDK outside the source tree. This uses Emscripten's
[CMake toolchain](https://emscripten.org/docs/compiling/Building-Projects.html)
through a small wrapper that diagnoses missing or mismatched installations.

```sh
git clone https://github.com/emscripten-core/emsdk.git /path/to/emsdk
git -C /path/to/emsdk checkout 5eb0bde7585670252e8ba05e9d361627bffd08b5
/path/to/emsdk/emsdk install 6.0.9
/path/to/emsdk/emsdk activate 6.0.9
. /path/to/emsdk/emsdk_env.sh

cmake --preset wasm-debug
cmake --build --preset wasm-debug
ctest --preset wasm-debug

cmake --preset wasm-release
cmake --build --preset wasm-release
ctest --preset wasm-release
```

The SDK tag is `6.0.9`, commit
`5eb0bde7585670252e8ba05e9d361627bffd08b5`. Its release mapping pins compiler
artifacts to `f04ea239d533260dd1db760dd2d668d5f9a88d6b`.
The activated SDK supplies Node through `EMSDK_NODE`; otherwise an executable
`node` or `nodejs` must be on PATH at configure time. On Windows activate
with `emsdk_env.bat` and select an
installed Make-compatible generator (or override with `-G Ninja`). When
switching SDK or runtime, remove only the affected `build/wasm-*` directories
and configure again. A missing SDK, wrong version, or missing Node produces a
configure error with setup instructions.

CMake sets `CMAKE_CROSSCOMPILING_EMULATOR` to Node before creating executable
targets. CTest retains the single `all` registration and propagates the
runner's exit status. Artifacts are `build/wasm-<mode>/bin/sl_tests.js` and its
sibling `.wasm`; run the JavaScript file through Node to inspect individual
case output. Keep both files together.

## Baseline flags and memory

Both presets use C17, `C_EXTENSIONS OFF`, and the unmodified `silk_warnings`
target. SDK defaults supply `-g` in Debug and `-O3 -DNDEBUG` in Release. No
fast-math, SIMD, relaxed SIMD, pthreads, LTO, or graphics flags are added.
The Node test executable alone receives:

```text
-sENVIRONMENT=node -sALLOW_MEMORY_GROWTH=0 -sABORTING_MALLOC=0
-sINITIAL_MEMORY=268435456 -sSTACK_SIZE=1048576 -sEXIT_RUNTIME=1
Debug:   -sASSERTIONS=2 -sSTACK_OVERFLOW_CHECK=2
Release: -sASSERTIONS=0
```

The test module reserves 256 MiB with a 1 MiB stack, accommodating tests that
construct multiple worlds and the maximum-capacity allocation checks. Ordinary
allocator exhaustion returns null. This is a test budget, separate from the
browser package's planned configurable 64 MiB default. No engine step gains
an allocator or platform dependency.

`compile_commands.json` records every compilation command; the Make generator's
`tests/CMakeFiles/sl_tests.dir/link.txt` records the exact link command. Capture
these, `emcc --version`, `node --version`, source revision/dirty status, and the
SDK commit alongside run results. Do not check machine-specific reports into
the repository. Compiler/linker studies must compare against this scalar
baseline and follow the frozen physical limits.

## wasm32 audit

The core stores entity counts and handle words as fixed-width integers.
`size_t` and pointers are 32 bits on wasm32; no pointer is a JS handle. Arena
carving uses 8-byte boundaries with compile-time alignment gates. Every
bounded body/tree slice fits wasm32; contact/joint slice multiplication and
summation check `SIZE_MAX`, and world creation rejects invalid capacities
before allocation. Existing exact budget tests cover small and maximum worlds,
including joint storage. Struct layout differences must be derived from the
actual pointer-bearing shell sizes, with the payload budgets kept exact.
In particular, the test's original 56-byte `sl_tree` header becomes 44 bytes
on wasm32 and occupies 48 aligned bytes. The test derives this difference
from its three pointers and eight uint32 fields, preserving the exact payload
totals and the existing 97/108 MiB maximum arena limits.

The full suites exercise finite input validation, pool exhaustion,
destroy/reuse/reset, deterministic replay, queries, memory accounting, and
allocation-free simulation behavior. Invalid direct stepping is a programmer
error in Debug; Release additionally tests its no-mutation fallback. The
forthcoming JS adapter must preflight these inputs before calling C.
