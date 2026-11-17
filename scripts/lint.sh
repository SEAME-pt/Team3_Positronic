#!/usr/bin/env bash
# ROS 2 Jazzy style checks on src/ (the ament_lint_common tools that colcon test runs).
#   bash scripts/lint.sh            check only (make format-check, CI)
#   bash scripts/lint.sh --fix      clang-format, then uncrustify reformat (make format)
#   VERBOSE=1 bash scripts/lint.sh  also print each tool's full output (make format-check V=1)
#
# Check mode prints one line per tool. A tool that fails also prints its problems, without the
# lines every tool prints for each file it checked. The last line says if everything passed.
# Exit code 0 when every tool passed, 1 when one failed.
set -u
cd "$(dirname "$0")/.."

CPP=(-name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp')
has() { [ -n "$(find src -type f \( "$@" \) -print -quit 2>/dev/null)" ]; }
count() { find src -type f \( "$@" \) 2>/dev/null | wc -l | tr -d ' '; }

if [ "${1:-}" = "--fix" ]; then
  has "${CPP[@]}" || { echo "No C/C++ files in src/"; exit 0; }
  find src -type f \( "${CPP[@]}" \) -print0 | xargs -0 clang-format -i
  # Exits 1 whenever it changed a file, so only its output matters here.
  ament_uncrustify --reformat src > /dev/null 2>&1
  echo "Formatted. Run make format-check to confirm."
  exit 0
fi

# Colour only on a terminal (NO_COLOR turns it off, FORCE_COLOR on).
if [ -z "${NO_COLOR:-}" ] && { [ -t 1 ] || [ -n "${FORCE_COLOR:-}" ]; }; then
  GREEN=$'\e[32m' RED=$'\e[31m' BOLD=$'\e[1m' DIM=$'\e[2m' RESET=$'\e[0m'
else
  GREEN='' RED='' BOLD='' DIM='' RESET=''
fi

# What every tool prints for each file it checked, and its tallies: hidden, the problems stay.
NOISE="^(No code style divergence|Done processing|checking |Using '--root|\* src/|Checked files:"
NOISE+="|[0-9]+ files checked|No problems found|[0-9]+ +[A-Z][0-9]+ |'[A-Z]'-type errors"
NOISE+="|Category '|Total errors found|[0-9]+ errors|[0-9]+ files with code style divergence)"

# The number of problems a tool reported, from its last lines (empty when it has none).
problems() {
  grep -oE "Total errors found: [0-9]+|^[0-9]+ errors|^[0-9]+ files with code style divergence" \
    "$1" | grep -oE '[0-9]+' | tail -n 1
}

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
failed=()
table=()

# run <tool> <what it checks>: one line with the result, the problems kept for the end.
# Verbose: the tool's full output now, its line in the table printed after every tool ran.
run() {
  local tool=$1 what=$2 out="$TMP/$1" start=$EPOCHREALTIME
  "ament_$tool" src > "$out" 2>&1
  local rc=$? took line
  took=$(awk -v a="$start" -v b="$EPOCHREALTIME" 'BEGIN { printf "%.1f s", b - a }')
  if [ "$rc" -eq 0 ]; then
    line=$(printf '  %s✔%s  %-11s %-24s %s%s%s' "$GREEN" "$RESET" "$tool" "$what" \
      "$DIM" "$took" "$RESET")
  else
    local n
    n=$(problems "$out")
    failed+=("$tool${n:+ ($n)}")
    line=$(printf '  %s✘%s  %-11s %-24s %s%s%s' "$RED" "$RESET" "$tool" "$what" \
      "$RED" "${n:-?} problem$([ "${n:-}" = 1 ] || echo s)" "$RESET")
  fi
  if [ "${VERBOSE:-}" ]; then
    echo "${BOLD}── $tool: full output ──${RESET}"
    cat "$out"
    echo
    table+=("$line")
  else
    echo "$line"
  fi
}

ncpp=$(count "${CPP[@]}")
npy=$(count -name '*.py')
header="${BOLD}Format check${RESET} ${DIM}src/: $ncpp C/C++ files, $npy Python files${RESET}"
if [ -z "${VERBOSE:-}" ]; then
  printf '\n%s\n\n' "$header"
fi
if [ "$ncpp" -gt 0 ]; then
  run uncrustify "C++ layout"
  run cpplint "C++ style"
  run cppcheck "C++ static analysis"
fi
if [ "$npy" -gt 0 ]; then
  run flake8 "Python style"
  run pep257 "Python docstrings"
fi
if [ $((ncpp + npy)) -gt 0 ]; then
  run copyright "License headers"
fi

# Verbose: the full outputs are above, so the summary comes in one piece after them.
if [ "${VERBOSE:-}" ]; then
  printf '\n%s\n\n' "$header"
  printf '%s\n' "${table[@]}"
fi

if [ ${#failed[@]} -eq 0 ]; then
  echo
  echo "${BOLD}${GREEN}✔ PASSED: every tool found no problems${RESET}"
  exit 0
fi

for entry in "${failed[@]}"; do
  tool=${entry%% *}
  echo
  echo "${BOLD}${RED}── $tool ──${RESET}"
  # Paths from src/, one blank line at most between problems, none before or after them.
  grep -vE "$NOISE" "$TMP/$tool" | sed 's#/ws/src/#src/#g' \
    | awk 'NF { if (seen && blank) print ""; print "    " $0; seen = 1; blank = 0; next } { blank = 1 }'
done

echo
echo "${BOLD}${RED}✘ FAILED: $(IFS=,; echo "${failed[*]}" | sed 's/,/, /g')${RESET}"
for entry in "${failed[@]}"; do
  if [ "${entry%% *}" = uncrustify ]; then
    echo "${DIM}  uncrustify problems are fixed by: make format${RESET}"
  fi
done
exit 1
