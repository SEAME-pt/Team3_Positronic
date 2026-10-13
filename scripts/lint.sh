#!/usr/bin/env bash
# ROS 2 Jazzy style checks on src/ (the ament_lint_common tools that colcon test runs).
#   bash scripts/lint.sh        check only (make format-check, CI)
#   bash scripts/lint.sh --fix  clang-format, then uncrustify reformat (make format)

set -u
cd "$(dirname "$0")/.."

CPP=(-name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp')
has() { [ -n "$(find src -type f \( "$@" \) -print -quit 2>/dev/null)" ]; }

if [ "${1:-}" = "--fix" ]; then
  has "${CPP[@]}" || { echo "No C/C++ files in src/"; exit 0; }
  find src -type f \( "${CPP[@]}" \) -print0 | xargs -0 clang-format -i
  # Exits 1 whenever it changed a file, so only its output matters here.
  ament_uncrustify --reformat src > /dev/null 2>&1
  echo "Formatted. Run make format-check to confirm."
  exit 0
fi

failed=0
run() { echo "== $1"; "$@" || failed=1; }
if has "${CPP[@]}"; then
  run ament_uncrustify src
  run ament_cpplint src
  run ament_cppcheck src
fi
if has -name '*.py'; then
  run ament_flake8 src
  run ament_pep257 src
fi
if has "${CPP[@]}" -o -name '*.py'; then
  run ament_copyright src
fi
exit "$failed"
