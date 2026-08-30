# AGENTS.md

Operational instructions for agents modifying `silk`. The human-facing
engineering and workflow policy lives in [CONTRIBUTING.md](CONTRIBUTING.md);
this file repeats the constraints that must be available before any change.
Project direction and non-goals live in [ROADMAP.md](ROADMAP.md).

## Repository map

`include/silk/` contains public headers, `src/` the implementation, `tests/`
the self-contained test framework and suites, and `examples/sandbox/` the
optional raylib demo. The root `CMakeLists.txt` defines the static library
target `silk::silk`.

## Non-negotiable engine constraints

- Use portable ISO C17 with `C_EXTENSIONS OFF`. Core engine code has no
  third-party dependencies and must not use POSIX, GNU, or MSVC extensions;
  build tooling and examples are exempt from the dependency rule.
- Keep simulation deterministic and fixed-timestep. Never introduce wall-clock
  time, uninitialized reads, or iteration-order-dependent floating-point work
  into simulation results. Expect bitwise determinism only per platform and
  build; use explicit tolerances for cross-platform numerical equivalence.
- Never store NaN or infinity as ordinary simulation state. Validate external
  floats before mutation. Use `sl_feq` or `SL_EXPECT_NEAR` for approximate
  comparisons; use exact float equality only when exactness is the property
  under test.
- Keep bulk entity state SoA. AoS is acceptable for transient structures such
  as contact manifolds when the whole record is consumed together.
- Allocate runtime memory during initialization and keep simulation steps
  allocation-free. Bound loops, counts, pools, queues, and work per step;
  avoid recursion.
- Preserve a path to SIMD, threading, GPU, and WASM, but do not add speculative
  backends, rendering abstractions, generic containers, or dispatch layers.

## Coding rules

Apply [TigerStyle](https://tigerstyle.dev/) through these repository rules:

- Keep interfaces small and deterministic. Prefer explicit data flow and a
  plain success/failure result over unnecessary indirection or rich status
  types. Use fixed-width integers for stored state and bounded counts.
- In hot paths, avoid needless copies, keep access cache-friendly, and record
  the expected memory/compute cost when a change materially affects layout or
  complexity.
- Prefix public APIs with `sl_` and use `snake_case`. Use established domain
  abbreviations only (`vec2`, `mat2`). Give related names equal-length words
  where practical so they align. Use big-endian naming:
  `connection_count_max`, `vec2_length_sq`, and `SL_BODY_COUNT_MAX`.
- Comments document contracts the type system cannot express: units, frames,
  sign conventions, valid ranges, NaN behavior, lifetimes, and invariants.
  Tuned constants need a derivation, source, or test that pins them.
- Use `SL_ASSERT` for programmer errors and internal invariants. Validate
  recoverable external input and return failure without partial mutation.
  Assertion expressions must have no side effects, and code must remain
  correct and warning-clean when `NDEBUG` removes them.
- Treat the `silk_warnings` target in `CMakeLists.txt` as authoritative for the
  warning set. Keep every first-party C target warning-clean; never weaken or
  suppress a warning merely to pass CI.
- When changing the version, update both `project(silk VERSION ...)` in
  `CMakeLists.txt` and `SL_VERSION_{MAJOR,MINOR,PATCH}` in
  `include/silk/silk.h`.

## Build and verification

The default local check is:

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Apply `clang-format` 22.1.3 to every touched `.c` and `.h` file. CI checks all
first-party C sources and headers using the pinned version.

Run checks in proportion to the change:

- Engine or test changes: run the debug build and tests.
- Assertion-sensitive or `NDEBUG`-dependent changes: also run the release
  build and tests.
- Memory, arena, pointer, geometry, or numerical changes: also run the
  `sanitize` preset.
- Sandbox changes, or public-header changes that affect it: build both
  `sandbox` and `sandbox-release`. The sandbox requires network access while
  configuring raylib and is compile-checked rather than executed in CI.

Never weaken assertions, tests, warnings, tolerances, or explicit limits to
make a change pass. If a check is wrong, fix it explicitly and explain why.

## Tests

The test runner is `tests/silk_test.h`; it has no per-test filter. Use
`SL_EXPECT`, `SL_EXPECT_INT_EQ`, and `SL_EXPECT_NEAR`. Randomized and property
tests must use a deterministic PRNG with a seed committed to source; never use
wall-clock seeding, `rand()`, or `time()`.

Adding a suite requires all four steps:

1. Add `tests/test_<name>.c` with static cases, a `k_cases[]` table, and an
   `int sl_<name>_suite(void)` runner returning the failure count.
2. Add the source to `sl_tests` in `tests/CMakeLists.txt`. Keep the single CTest
   registration; per-suite registrations would rerun the whole binary.
3. Declare the runner in `tests/suites.h`.
4. Call the runner from `main()` in `tests/sl_tests.c` and sum its failures.

## Repository workflow

Do not create branches, commits, issues, or pull requests unless explicitly
requested. When requested, follow [CONTRIBUTING.md](CONTRIBUTING.md): use
Conventional Commits, a short-lived `<type>/<topic>` branch, the pull request
template, and the issue forms.
