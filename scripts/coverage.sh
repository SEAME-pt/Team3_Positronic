#!/usr/bin/env bash
# Line coverage of src/, C++ and Python combined into one lcov tracefile (#46).
#   bash scripts/coverage.sh    (make coverage, CI)
#
# Builds the workspace a second time with --coverage in build_cov/ and install_cov/, never in
# build/ and install/: instrumented binaries are slower and must not reach the Jetson.
# Output in coverage/: cpp.info, py.info and total.info, the file the gate reads.
#
# What is counted, and what is excluded (also listed in .github/workflows/ci.yml):
#   counted    C++ and Python under src/, including files no test opens (at 0 %)
#   */test/*   test code covering itself would inflate the number
#   */launch/* configuration, checked by running the simulation (.coveragerc)
#   */setup.py packaging, not robot code (.coveragerc)
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD=build_cov
INSTALL=install_cov
OUT=coverage
SRC="$(pwd)/src"

command -v lcov > /dev/null || { echo "lcov is missing: run make image" >&2; exit 1; }
python3 -c 'import pytest_cov' 2> /dev/null \
  || { echo "pytest-cov is missing: run make image" >&2; exit 1; }

# lcov 2.x options:
#   mismatch,mismatch   GCC 13 and lcov disagree on the end line of functions made by GoogleTest's
#                       TEST() macro. Only test files are affected, and they are removed below.
#                       Doubled, so the warnings are silenced too.
#   geninfo_unexecuted_blocks=1  silences "unexecuted block on non-branch line" in the C++
#                       standard library headers, which are not counted either.
LCOV=(lcov --quiet --ignore-errors mismatch,mismatch --rc geninfo_unexecuted_blocks=1)

rm -rf "${OUT}"
mkdir -p "${OUT}"

echo "== Build with --coverage (${BUILD}/)"
colcon build --build-base "${BUILD}" --install-base "${INSTALL}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=--coverage -DCMAKE_C_FLAGS=--coverage

# Counts from an earlier run would add to this one.
"${LCOV[@]}" --zerocounters --directory "${BUILD}"
rm -f "${BUILD}/.coverage"

# Every instrumented line at zero, so a file no test runs still counts (at 0 %), instead of
# disappearing and making the number look better than it is.
"${LCOV[@]}" --capture --initial --directory "${BUILD}" --output-file "${OUT}/cpp_base.info"

# colcon starts pytest itself, so the coverage options reach it through PYTEST_ADDOPTS.
# --cov-append: each package's tests add to one data file. Sequential: no two packages write it
# at once. The C++ counters are written by the tests as they run.
echo "== Tests with coverage"
COVERAGE_FILE="$(pwd)/${BUILD}/.coverage" \
PYTEST_ADDOPTS="--cov --cov-config=$(pwd)/.coveragerc --cov-append --cov-report=" \
  colcon test --build-base "${BUILD}" --install-base "${INSTALL}" --executor sequential
colcon test-result --test-result-base "${BUILD}" || echo "WARNING: some tests failed" >&2

echo "== C++"
"${LCOV[@]}" --capture --directory "${BUILD}" --output-file "${OUT}/cpp_test.info"
"${LCOV[@]}" -a "${OUT}/cpp_base.info" -a "${OUT}/cpp_test.info" -o "${OUT}/cpp.info"
"${LCOV[@]}" --extract "${OUT}/cpp.info" "${SRC}/*" -o "${OUT}/cpp.info"
"${LCOV[@]}" --remove "${OUT}/cpp.info" '*/test/*' -o "${OUT}/cpp.info"
lcov --summary "${OUT}/cpp.info"

echo "== Python"
python3 -m coverage lcov --rcfile=.coveragerc --data-file="${BUILD}/.coverage" \
  -o "${OUT}/py.info"

echo "== Total"
"${LCOV[@]}" -a "${OUT}/cpp.info" -a "${OUT}/py.info" -o "${OUT}/total.info"
lcov --summary "${OUT}/total.info"
