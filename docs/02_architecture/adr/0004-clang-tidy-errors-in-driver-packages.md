# 0004. clang-tidy warnings are errors in the driver packages

2026-11-03, proposed, Felipe Suassuna (issue #38).

## Context

`.clang-tidy` has `WarningsAsErrors: ''`, so clang-tidy only warns. CONTRIBUTING.md asks for "no
new warnings", but nothing enforced it, and CI did not run clang-tidy at all. The driver is the
code that moves the arm and has to be defended in the oral review.

## Decision

- CI runs `make tidy-strict` after the build and the tests. It runs `run-clang-tidy` on
  `positronic_hardware` and `positronic_protocol` with `-warnings-as-errors="*"`, so any warning
  fails the PR.
- The rule lives on the command line (the Makefile), not in `.clang-tidy`, so `make tidy` stays a
  soft check everywhere else.
- The full run is kept on every PR. Measured: 72 s of wall time for all of `src/` in the dev VM.
- The baseline was 13 warnings, all fixed before the gate was switched on (PR #96).

## Alternatives

- **`WarningsAsErrors: '*'` in `.clang-tidy`:** one line, but it applies to every package and to
  the editor, and makes experiments in new packages fail.
- **A `.clang-tidy` per package:** the same effect, but the rules end up in several places that
  can drift apart.
- **Only check the files a PR changes:** faster, but a change to a header does not re-check the
  `.cpp` files that include it. Not needed while the full run takes about a minute.
- **Fail only on new warnings (diff against dev):** needs extra tooling. With a baseline of 0 it
  gives nothing.
- **Keep it a review rule:** a warning ignored once is ignored forever.

## Consequences

- A PR touching the driver cannot merge with a clang-tidy warning. A wrong warning needs a
  `// NOLINT(check)` with a reason.
- CI takes longer by the duration of the tidy step.
- A new C++ package is not strict until it is added to `TIDY_STRICT_FILES`.

## Revisit when

- The tidy step takes more than 5 minutes in CI: check only the changed files on PRs and run the
  full check on pushes to `dev`.
- Other packages have C++ and are clean: make all of `src/` strict.
