# 0005. Line coverage gate: 80 % per package, report-only until #46

2026-11-10, proposed, Felipe Suassuna (issue #109).

## Context

#46 asks for a coverage gate at 80 %. Nothing measured coverage before, so nobody knew if the
gate would pass. If it were switched on blocking, it could stop every PR until the missing tests
were written.

First measurement (`make coverage`, dev at 4101cde, 415 tests):

| Package | Lines | Coverage |
| --- | ---: | ---: |
| `positronic_bringup` | 44 / 81 | 54.3 % |
| `positronic_hardware` | 457 / 501 | 91.2 % |
| `positronic_mission` | 0 / 49 | 0.0 % |
| `positronic_moveit_config` | 50 / 89 | 56.2 % |
| `positronic_protocol` | 131 / 131 | 100.0 % |
| **Total** | **682 / 851** | **80.1 %** |

The total passes 80 %, but `positronic_mission` has no tests at all. One number for the whole
workspace lets a well-tested package hide one without tests.

## Decision

- **Line coverage**, C++ (gcov and lcov) and Python (pytest-cov), combined into one lcov tracefile
  (`scripts/coverage.sh`, `make coverage`).
- **Each package must reach 80 %** (`scripts/coverage_gate.py`). The total is shown for
  information only. A package's number is lines hit / lines found, summed over its files, not an
  average of file percentages.
- **Files no test runs count, at 0 %.** C++: an lcov `--initial` baseline. Python: `source` in
  `.coveragerc`, and the gate counts any `.py` file coverage.py still missed at 0 %, with
  coverage.py's own statement count, so an untested Python package fails like a C++ one.
- **Excluded:** `*/test/*` (test code covering itself), `*/launch/*` (configuration, checked
  by running the simulation) and `*/setup.py` (packaging). Generated code (`positronic_interfaces`) is in `build_cov/`, not
  `src/`, so it doesn't count either. Everything else under `src/` counts.
- **Report-only until #46 (Sprint 03):** the CI step has `continue-on-error: true`. Below 80 %
  the step is red, but the job stays green. The table goes to the job summary page. #46 writes
  the missing tests (`positronic_mission`, `positronic_bringup`, `positronic_moveit_config`) and
  then removes `continue-on-error`.
- The coverage build has its own `build_cov/` and `install_cov/`, so instrumented binaries never
  mix with `build/` or reach the Jetson. `lcov` and `python3-pytest-cov` are in the `tools` stage
  of the Dockerfile (ci and dev images, not runtime).

## Alternatives

- **One gate on the total:** simpler, but with today's numbers it passes while a package has 0 %.
- **Blocking from the start (option A in #46):** every PR fails until the missing tests exist, so
  the team would lower the threshold or skip the gate.
- **Write the tests first, then switch on the gate (option C):** the end state, but it needed a
  measurement first, to know which tests were missing. Planned for Sprint 03 (#46).
- **Function coverage:** a function counts as covered after one line of it runs, so it tells
  less than line coverage. lcov reports it for C++ (88.1 %), but the Python tool doesn't.
- **Branch coverage:** stronger than line coverage, because it checks that each `if` was tested
  both ways. Left out for now to keep the first gate simple (see "Revisit when").
- **An HTML report (genhtml, uploaded as a CI artifact):** more to maintain. The table in the
  job summary and the worst-files list are enough to find what to test.
- **Counting launch files:** they are configuration (which nodes start, with which parameters),
  not logic a unit test can check. Running the simulation checks them.

## Consequences

- Every CI run builds and tests `src/` a second time, with coverage. That's about the time of the
  Build and Test steps together.
- Until #46, a package can drop below 80 % without blocking a merge. The red step and the job
  summary make it visible.
- A new package with code is judged from its first PR. A small package swings a lot: in a 49-line
  package, 10 lines are 20 %.
- Coverage shows which code ran, not that the tests checked it. A test without assertions still
  raises the number. Reviews still have to read the tests.
- `.coveragerc` uses the dev container's path (`/ws/src`), so `make coverage` must run in the
  container (as `make` does by default).

## Revisit when

- #46 is done and every package is at 80 %: remove `continue-on-error`.
- The coverage step makes CI too slow: run it only on pushes to `dev`, or reuse the Build step's
  output.
- A package's code mostly runs only with ROS (nodes that need a live graph): decide per package
  if a lower threshold or launch tests are the better answer.
- The line gate holds: add branch coverage to the report, and consider mutation testing (`mutmut`
  for Python, `mull` for C++) to measure whether the tests check what they run.
