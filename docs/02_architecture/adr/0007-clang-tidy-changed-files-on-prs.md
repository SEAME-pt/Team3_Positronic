# 0007. clang-tidy: changed files on PRs, everything on dev, bugprone only on tests

2026-11-18, proposed, Felipe Suassuna (issue #144). Supersedes [0004](0004-clang-tidy-errors-in-driver-packages.md).

## Context

0004 kept the full `make tidy-strict` run on every PR, and said to revisit it when the step takes
more than 5 minutes in CI. It does now: **342 s** median over 20 runs (300 to 566 s), 42 % of a
13.6 min PR run. When 0004 was written it took 72 s; since then `positronic_safety` became strict
and the tests grew.

Measured (#144, clang-tidy 18, one file at a time unless said otherwise):

- 32 strict files: 10 in `src/`, 22 in `test/`. The test files are **74 %** of the CPU time
  (1004 of 1359 CPU-s).
- Most of it is `clang-analyzer-*` on the GoogleTest bodies: `test_load_robot_system.cpp` takes
  68 s with every check, 20 s without the analyzer, 2.5 s to parse.
- With only `bugprone-*`, the 22 test files take 99 CPU-s instead of 654 (−85 %), 30 s instead of
  192 s 4 at a time (dev VM). Every one of them passes.
- 11 of the last 20 merged PRs changed no strict C++ file.
- A PR that changes `positronic_protocol/types.hpp` (included across packages) affects 14 of the
  32 files; one `.cpp` affects 1 (E4 runs on PR #158).

## Decision

- **PRs check only the strict files they can affect** (`make tidy-changed`,
  `scripts/tidy_changed.py`): the changed `.cpp` files; every file that includes a changed header
  (from the CMake depfiles `build/*/CMakeFiles/**/*.o.d`); the whole package for a changed
  `CMakeLists.txt` or `package.xml`; every file that includes generated headers for a changed
  `.msg`, `.srv` or `.action`. Everything when `.clang-tidy`, `docker/Dockerfile`, the script or a
  `TIDY_*` variable in the Makefile changes.
- **Every push to `dev` (and `main`) runs everything** (`make tidy-strict`).
- **Test files (`<package>/test/`) get only the `bugprone-*` checks** that `.clang-tidy` enables
  (team call, 2026-11-18). `src/` keeps the full set.
- Unchanged from 0004: the strict packages, every warning an error, and the rule on the command
  line (Makefile), not in `.clang-tidy`.

## Alternatives

- **Keep the full run on every PR (0004):** about 6 minutes, the longest job by far once the
  checks run in parallel.
- **Split the full run over 3 runners** (E5, PR #155): 349 s → 194 s median, not the 120 s
  expected, because `test_load_robot_system.cpp` alone takes 165 to 223 s on a shared runner.
  Three times the runners for a smaller gain than dropping the analyzer on tests.
- **No clang-tidy on tests at all:** also drops `bugprone-*`, which catches real bugs in test
  code too (use after move, wrong `sizeof`, a suspicious semicolon that skips an assertion).
- **A `.clang-tidy` in each `test/` folder:** the editor would show the same as CI, but it is
  four copies that drift apart, the reason 0004 gave against per-package files.
- **Fail only on new warnings:** needs extra tooling, and with a baseline of 0 it gives nothing.

## Consequences

- A PR that changes no strict file spends no time in clang-tidy; one header used across
  packages costs about 2 minutes, before the test files got lighter.
- A warning in a file the selection misses reaches `dev`; the full run on the next push to `dev`
  catches it, right after the merge. Known gaps: a `CMakeLists.txt` change that alters flags of
  the packages that depend on it, and, locally, a new file not yet `git add`ed.
- No style, `modernize`, `readability` or analyzer warnings fail CI in `test/`. The editor
  (clangd) still shows them there: they are advice.
- A new check family in `.clang-tidy` must also be switched off in `TIDY_TEST_CHECKS`;
  `scripts/test_tidy_changed.py` fails until it is.
- `make tidy-changed` runs git inside the dev container, so it doesn't work in a git worktree
  (`make tidy-strict` does).

## Revisit when

- A warning reaches `dev` that the PR's selection should have caught: fix the rule, or go back to
  the full run on PRs.
- The full run on `dev` takes more than 5 minutes again: shard it over runners (E5).
- A bug in a test that one of the dropped checks would have flagged.
