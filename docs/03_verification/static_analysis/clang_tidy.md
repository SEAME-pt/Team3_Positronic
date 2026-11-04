# clang-tidy in CI (#38)

Static analysis on the packages that talk to the arm or command it. A warning there fails CI.

## Commands

| Command | What it does |
|---|---|
| `make tidy` | clang-tidy on all of `src/`. Warnings are printed, nothing fails. Use it while you work. |
| `make tidy-strict` | The same tool on the strict packages only (`TIDY_STRICT_FILES` in the Makefile), with `-warnings-as-errors="*"`. CI runs it after the build. |

Both need `make build` first: clang-tidy reads `build/compile_commands.json` to get the include
paths and flags of each file. To check one file while you work:
`run-clang-tidy -quiet -p build src/positronic_hardware/src/serial_arm.cpp`.

cppcheck already runs in CI inside `make format-check` (`scripts/lint.sh`) and again per package
in `make test`.

## Which packages are strict

- `positronic_hardware` and `positronic_protocol`: the code that talks to the arm.
- `positronic_mission`: the node that commands the arm and publishes the heartbeat the safety
  supervisor watches. Added on 2026-11-04, agreed with its author, with a baseline of 0 warnings.

`positronic_interfaces` only has message definitions, so there is no C++ to check. When a new C++
package should be strict (the safety supervisor, for example), add it to `TIDY_STRICT_FILES`.

## Baseline (dev, 2026-11-03, before the fixes)

13 warnings in `positronic_hardware` and `positronic_protocol`, all in `positronic_hardware`.
`positronic_protocol` had none.

| Check | Count | Where | Fix |
|---|---|---|---|
| `concurrency-mt-unsafe` | 5 | `strerror` in `PosixSerialPort` | `std::system_error`: same message, thread-safe |
| `cppcoreguidelines-pro-bounds-pointer-arithmetic` | 2 | `reply_assembler.hpp`, `PosixSerialPort::write` | `std::next(ptr, n)` |
| `misc-const-correctness` | 3 | `test_serial_arm.cpp` | `const SerialArm arm` |
| `readability-redundant-member-init` | 2 | `reply_assembler.hpp`, `test_serial_arm.cpp` | drop the `{}` |
| `cppcoreguidelines-special-member-functions` | 1 | `PosixSerialPort` | delete copy and move (it owns a file descriptor) |

Fixed in PR #96. The baseline in the strict packages is now **0 warnings**.

The "N warnings generated" lines in the output come from system and ROS headers. clang-tidy
suppresses them (`HeaderFilterRegex` in `.clang-tidy` keeps only ours). They are not ours to fix.

## Cost

Measured on 2026-11-03 in the dev VM, `make tidy` over all of `src/`:

| Measure | Value |
|---|---|
| Wall time | 72 s |
| CPU time | 201 s |
| Checks on one file (`--enable-check-profile`, `serial_arm.cpp`) | 2.6 s |

Most of the time is spent parsing the ROS and standard headers, not running the checks, so
turning checks off would save little.

In CI: TODO fill in the duration of the "Static analysis (clang-tidy)" step from the first run.

## If a warning is wrong

Silence that one line with `// NOLINT(check-name)` and a reason in the PR (see CONTRIBUTING.md).
Do not turn off a check for the whole project to get CI green.
