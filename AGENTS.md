# AGENTS.md

Agent guidance for `silk`: a portable, data-oriented physics/simulation engine in pure C17. Coding law: [TigerStyle](https://tigerstyle.dev/) (safety, performance, experience; spirit of NASA's Power of Ten).

## Layout

`include/silk/` public headers · `src/` implementation · `tests/` self-contained test framework · `examples/sandbox/` demo app. Single static library target `silk::silk`, wired from the root `CMakeLists.txt`.

## Build & test

```sh
cmake --preset debug         # configure (presets: debug, release, sanitize); needs CMake >= 3.28
cmake --build --preset debug # builds lib + tests + sandbox
ctest --preset debug         # or run ./build/debug/tests/sl_tests directly
```

Formatting is clang-format (`.clang-format`, LLVM-based, 4-space indent); run `clang-format -i` on every touched C file before committing — CI checks formatting with a pinned clang-format version (22.1.3), so unformatted code fails the `format` job. Warning-clean compilation (below) is the quality gate, enforced by CI (GitHub Actions: gcc + clang on Linux, macOS, and an ASan+UBSan sanitizer job).

## Hard constraints

- **C17 with `C_EXTENSIONS OFF`**: portable ISO C only. No POSIX, no GNU/MSVC extensions, no third-party libraries — the core `silk` library has zero dependencies; build tooling and examples are exempt.
- The `silk` library compiles with `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion -Wformat=2 -Wundef -Wmissing-prototypes -Wstrict-prototypes -Werror` (MSVC `/W4 /WX`). Only the lib target enforces this, but all engine code must stay warning-clean.
- The version is duplicated: `project(silk VERSION ...)` in `CMakeLists.txt` and `SL_VERSION_{MAJOR,MINOR,PATCH}` in `include/silk/silk.h`. Bump both together.
- Simulation must stay **deterministic, fixed-timestep**. Bulk entity state is SoA (struct-of-arrays); transient solver structures (e.g., contact manifolds) may be AoS when that is the natural fit. Never introduce wall-clock time, uninitialized reads, or iteration-order-dependent floating point into simulation results. Bitwise determinism holds per platform and build; cross-platform or future GPU results are held to numerical equivalence within explicit tolerances, not bit-identical floats.
- Simulation state must remain **finite and numerically sound**: never store NaN or infinity as ordinary state, validate externally supplied floats, compare with explicit tolerances (`sl_feq`, `SL_EXPECT_NEAR`) — no exact float equality unless exactness is the property under test.
- Design data layouts and APIs so future SIMD, threading, GPU, and WASM ports stay possible, but never introduce GPU or rendering abstractions speculatively — CPU correctness first.

## Testing

The test runner is a hand-rolled header (`tests/silk_test.h`), not an external framework. There is no per-test filter; assertions are `SL_EXPECT`, `SL_EXPECT_INT_EQ`, `SL_EXPECT_NEAR`. Randomized/property tests must use seeded deterministic PRNGs with the seed committed to source — never seed from wall-clock time (`rand()`, `time()`), or failures stop being reproducible.

Adding a suite requires touching four files — missing any of the last two means the suite silently never runs:

1. New `tests/test_<name>.c`: static case functions, a `k_cases[]` array, and `int sl_<name>_suite(void)` returning the failure count.
2. Add the file to the `sl_tests` executable in `tests/CMakeLists.txt`. ctest registration stays a single `add_test(NAME all COMMAND sl_tests)` — the runner executes every suite, so per-suite entries would re-run the whole binary.
3. Declare the runner in `tests/suites.h`.
4. Call the runner from `main()` in `tests/sl_tests.c` and sum its failures.

## TigerStyle rules for this codebase

Safety:
- Put an explicit limit on everything: bound loops, queues, and counts; avoid recursion. Detect runaway execution instead of trusting it.
- Allocate all memory up front (pools/arenas) in the simulation runtime; never allocate during simulation. Non-runtime tooling and examples may allocate freely. Prefer fixed-width types (`uint32_t`) over `int`/`long`.
- Use assertions for arguments, returns, and invariants — check both the expected and the unexpected. Fail loudly rather than continue corruptly.
- Keep interfaces small and deterministic: minimize surface area, simplify signatures (a plain success/failure beats rich return values), push control flow up and data flow down.
- No abstraction without a demonstrated need: no interfaces, dispatch layers, generic containers, or indirection until a concrete requirement justifies them — a single `sl_solver_step(...)` beats an `ISolver` hierarchy.

Performance:
- Sketch costs before building (memory vs compute, bandwidth vs latency). Performance is decided in the design phase, not after profiling.
- Zero-copy and cache-friendly in hot paths: avoid needless copying, align structs to their largest field, batch work into large units.

Experience/naming:
- Public API prefix `sl_`; `snake_case`; no abbreviations except established domain terms (`vec2`, `mat2`); related names use equal-length words so they align.
- Big-endian naming — most significant word first: `connection_count_max`, not `max_connection_count`; `vec2_length_sq`, not `sq_length`. Capacity macros follow suit: `SL_BODY_COUNT_MAX`, never `SL_MAX_BODIES`.
- Comments state contracts the code can't express — units, frames, sign conventions, valid ranges, NaN behavior, invariants — never structure, narration, or what the signature already says.
- Tuned constants carry a trail: derivation, source, or the test that pins them (`beta = 0.2f` at 60 Hz, bounded by tests/stacking). A bare magic number is debt.
- No technical debt: do it right the first time. Prefer simple, elegant structures over clever ones.

## Workflow

Conventional Commits (`feat:`, `fix:`, `test:`, `chore:`), optionally scoped by subsystem (`feat(math): ...`, `fix(collision): ...`); short-lived `feat/*` branches PR'd into `main`. Pull requests follow `.github/PULL_REQUEST_TEMPLATE.md`; issues go through the forms under `.github/ISSUE_TEMPLATE/`.

Run the relevant tests after every change. Never weaken assertions, tests, warnings, or limits to make a change pass; if a check is wrong, fix the code or the check explicitly and say why.
