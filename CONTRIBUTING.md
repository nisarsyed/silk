# Contributing to Silk

Silk is a portable, data-oriented physics and simulation engine. Contributions
follow [TigerStyle](https://tigerstyle.dev/): safety first, performance designed
up front, and interfaces made clear for their callers. This document is the
human-facing source of truth for engineering and repository workflow.
Operational guidance for coding agents is in [AGENTS.md](AGENTS.md).

## Engineering principles

### Safety and determinism

- Write portable ISO C17. The core library must have no third-party
  dependencies and must not rely on POSIX, GNU, or MSVC extensions.
- Keep simulation fixed-timestep and deterministic for a given call sequence.
  Wall-clock time belongs above the simulation API.
- Keep stored simulation state finite. Validate external values before
  mutation and reject invalid input without partially changing state.
- Bound every count and unit of work. Avoid recursion and unbounded queues or
  loops. Allocate pools and arenas during initialization; simulation steps must
  not allocate.
- Use `SL_ASSERT` for programmer errors and internal invariants, not recoverable
  user input. Assertions compile out under `NDEBUG`, so their expressions must
  have no side effects and cannot be the only use of a value.
- Do not weaken assertions, tests, compiler warnings, tolerances, or explicit
  limits to make a change pass. Correct a faulty check explicitly and explain
  the reason.

### Data and performance

- Keep bulk entity state in struct-of-arrays form. A transient structure may
  use array-of-structs when consumers naturally need the complete record.
- Prefer zero-copy, contiguous, cache-friendly access in hot paths. Keep
  structures naturally aligned to their largest member and batch work into
  large units where practical.
- Before materially changing a hot loop or data layout, sketch its memory,
  bandwidth, and compute costs. Add complexity only for a demonstrated current
  requirement.
- Keep the CPU implementation correct first. Preserve future SIMD, threading,
  GPU, and WASM options without adding speculative backends or rendering
  abstractions.

### APIs, naming, and comments

- Prefix public APIs with `sl_` and use `snake_case`. Use abbreviations only
  when established in the domain, such as `vec2` and `mat2`. Prefer fixed-width
  integers such as `uint32_t` over `int` or `long` for stored state and counts.
- Give related names equal-length words where practical so declarations and
  uses align visually.
- Use big-endian naming, with the most significant word first:
  `connection_count_max`, not `max_connection_count`; `vec2_length_sq`, not
  `sq_length`; `SL_BODY_COUNT_MAX`, not `SL_MAX_BODIES`.
- Keep interfaces small and deterministic. Prefer explicit control flow and
  simple success/failure results over generic containers, dispatch layers, or
  indirection without a concrete need.
- Comments record contracts the code cannot express: units, coordinate frames,
  sign conventions, valid ranges, NaN behavior, pointer lifetimes, and
  invariants. Do not narrate the implementation.
- Give tuned constants a trail: a derivation, a source, or a test that pins the
  value. Do not leave unexplained magic numbers.
- Use approximate float comparisons (`sl_feq`, `SL_EXPECT_NEAR`) unless exact
  representation is deliberately the property being tested.

## Build and test

Silk requires CMake 3.28 or newer and a C17 compiler. Configure, build, and test
the default debug preset with:

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

The same three-command flow applies to `release` and `sanitize`. The optional
`sandbox` and `sandbox-release` presets fetch raylib and build the interactive
example. See [README.md](README.md) for sandbox requirements and usage.

The `silk_warnings` CMake target defines the authoritative warning set and
treats warnings as errors for first-party C targets. Format every touched `.c`
and `.h` file with clang-format 22.1.3; CI uses that exact version.

Use this verification matrix:

| Change | Required checks |
| --- | --- |
| Engine or tests | Debug configure, build, and CTest |
| Assertions or `NDEBUG` behavior | Debug and release tests |
| Memory, arenas, pointers, geometry, or numerics | Debug and sanitizer tests |
| Sandbox or public headers affecting it | `sandbox` and `sandbox-release` builds |
| Documentation only | Check links, commands, and consistency with source files |

Record the exact checks and results in the pull request.

## Test framework

Tests use the self-contained `tests/silk_test.h` runner rather than an external
framework. Assertions are `SL_EXPECT`, `SL_EXPECT_INT_EQ`, and
`SL_EXPECT_NEAR`. Randomized or property tests must use a deterministic PRNG
with a seed committed to source; never seed from wall-clock time.

The runner has no per-test filter. Adding a suite requires all four steps:

1. Add `tests/test_<name>.c` with static test cases, a `k_cases[]` table, and an
   `int sl_<name>_suite(void)` runner returning its failure count.
2. Add the source file to `sl_tests` in `tests/CMakeLists.txt`.
3. Declare the runner in `tests/suites.h`.
4. Call it from `main()` in `tests/sl_tests.c` and add its failures to the
   total.

Keep a single `add_test(NAME all COMMAND sl_tests)` registration. A CTest entry
per suite would rerun the complete test binary for every entry.

## Version changes

The version is intentionally duplicated. Update both locations together:

- `project(silk VERSION ...)` in `CMakeLists.txt`
- `SL_VERSION_MAJOR`, `SL_VERSION_MINOR`, and `SL_VERSION_PATCH` in
  `include/silk/silk.h`

## Git and review workflow

Use Conventional Commits, optionally scoped by subsystem:

```text
feat(math): add rotation composition
fix(collision): preserve manifold ordering
test(world): cover stale handle reuse
chore(build): update sanitizer configuration
```

Work on a short-lived `<type>/<topic>` branch, such as `feat/collision` or
`fix/shape-ray-cast`, and open it against `main`. Pull requests follow
`.github/PULL_REQUEST_TEMPLATE.md`; explain the motivation, important design
decisions, limitations, and exact verification performed. Issues should use the
forms under `.github/ISSUE_TEMPLATE/`.

Do not knowingly leave broken invariants or weakened guardrails as follow-up
work. When a legitimate follow-up is outside the current change, document its
scope and rationale explicitly.
