#!/usr/bin/env bash
# Positronic GPU test: does passing the GPU into the dev container work, and does it help Gazebo?
# It compares, on YOUR machine and with repeated runs (one run is too noisy to trust):
#   sw        software rendering (LIBGL_ALWAYS_SOFTWARE=1), the baseline
#   dri       /dev/dri passed in with numeric group ids  (Intel, AMD, VMs)   [if /dev/dri exists]
#   nvidia    NVIDIA Container Toolkit, gpus: all                            [if nvidia-ctk exists]
#   headless  Gazebo server only, no GUI: the physics-only RTF
# plus two quick checks that show why the groups are passed by number (dri-names, dri-nogroup).
#
#   bash scripts/gpu-test.sh            asks for your name
#   bash scripts/gpu-test.sh pedro      or pass it
#
# Run it in a normal terminal on your machine (NOT inside a container). It needs the dev image
# (make image) and a desktop session. It takes 6 to 12 minutes; a Gazebo window opens and closes
# many times: don't touch it, and close heavy apps first so the numbers mean something.
# It changes nothing: the containers are throwaway (--rm) and the override files go to /tmp.
# Result: scripts/gpu-test-<name>.txt. Commit and push that file (the script prints how).
set -u
cd "$(dirname "$0")/.." || exit 1

# Tunables (the defaults are what we want; the others are for testing the script itself).
REPS="${REPS:-3}"          # repetitions of each GUI mode
WARMUP="${WARMUP:-15}"     # seconds to let Gazebo settle before measuring
SAMPLES="${SAMPLES:-20}"   # RTF samples per run (about one per second)

if [ -e /.dockerenv ]; then echo "Run this on your machine, not inside a container."; exit 1; fi
if [ ! -f compose.yaml ]; then echo "compose.yaml not found: run this from inside the repo."; exit 1; fi
if ! docker image inspect "positronic-dev:${ROS_DISTRO:-jazzy}" > /dev/null 2>&1; then
  echo "The dev image is missing. Run 'make image' first."; exit 1
fi

NAME="${1:-}"
if [ -z "$NAME" ]; then read -r -p "Your name (used in the file name): " NAME; fi
NAME=$(printf '%s' "$NAME" | { iconv -f utf-8 -t ascii//TRANSLIT 2> /dev/null || cat; } \
  | tr '[:upper:]' '[:lower:]' | tr -c 'a-z0-9\n' '-' | sed 's/^-*//; s/-*$//' | cut -c1-30)
if [ -z "$NAME" ]; then echo "A name is needed."; exit 1; fi

OUT="scripts/gpu-test-${NAME}.txt"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
echo "Writing $OUT"
exec > >(tee "$OUT") 2>&1

sec() { printf '\n===== %s =====\n' "$*"; }
run() { printf '$ %s\n' "$*"; "$@" 2>&1; echo "[exit $?]"; }

echo "Positronic GPU test for ${NAME}, $(date -Is)  (REPS=$REPS WARMUP=$WARMUP SAMPLES=$SAMPLES)"

# ---------------------------------------------------------------- A. host
sec "A. Host"
run uname -m
run bash -c 'grep PRETTY_NAME /etc/os-release'
run bash -c 'echo "session=${XDG_SESSION_TYPE:-unset} DISPLAY=${DISPLAY:-unset} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-unset}"'
run nproc
run bash -c 'echo "loadavg: $(cut -d" " -f1-3 /proc/loadavg)"'
run bash -c 'lspci | grep -iE "vga|3d|display"'
run nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
run bash -c 'stat -c "%n gid=%g (%G)" /dev/dri/* 2>/dev/null'
run docker compose version
run bash -c 'docker info 2>/dev/null | grep -i runtimes'
run nvidia-ctk --version

HAVE_DRI=0; HAVE_NVIDIA=0
if ls /dev/dri/renderD* > /dev/null 2>&1; then HAVE_DRI=1; fi
if command -v nvidia-ctk > /dev/null && nvidia-smi -L > /dev/null 2>&1; then HAVE_NVIDIA=1; fi
echo "modes available: dri=$HAVE_DRI nvidia=$HAVE_NVIDIA"
[ "$HAVE_NVIDIA" = 0 ] && echo "nvidia mode skipped: needs nvidia-ctk and a working nvidia-smi on the host."

# Numeric group ids of the GPU device nodes: the host's numbers, not names (the names resolve to
# different numbers in the container).
export VIDEO_GID="$(stat -c %g /dev/dri/card* 2> /dev/null | head -n1)"
export RENDER_GID="$(stat -c %g /dev/dri/renderD* 2> /dev/null | head -n1)"
: "${VIDEO_GID:=44}" "${RENDER_GID:=44}"
export VIDEO_GID RENDER_GID

cat > "$TMP/dri.yaml" << 'EOF'
services:
  dev:
    devices:
      - /dev/dri:/dev/dri
    group_add:
      - "${VIDEO_GID}"
      - "${RENDER_GID}"
EOF
cat > "$TMP/dri-names.yaml" << 'EOF'
services:
  dev:
    devices:
      - /dev/dri:/dev/dri
    group_add:
      - video
      - render
EOF
cat > "$TMP/dri-nogroup.yaml" << 'EOF'
services:
  dev:
    devices:
      - /dev/dri:/dev/dri
EOF
cat > "$TMP/nvidia.yaml" << 'EOF'
services:
  dev:
    gpus: all
    environment:
      NVIDIA_DRIVER_CAPABILITIES: graphics,display,utility,compute
      # Laptops with two GPUs (PRIME): without these two, GLX picks Mesa, finds no Intel
      # device inside the container and falls back to zink; Gazebo then segfaults.
      __NV_PRIME_RENDER_OFFLOAD: "1"
      __GLX_VENDOR_LIBRARY_NAME: nvidia
EOF

# ---------------------------------------------------------------- what runs inside the container
cat > "$TMP/inner.sh" << 'INNER'
set +e
WARMUP="${WARMUP:-15}"; SAMPLES="${SAMPLES:-20}"
PAT='direct rendering|OpenGL renderer|OpenGL version|core profile version|rror|nable|ailed'

stats() {   # numbers on stdin -> n, min, median, max
  sort -g | awk '{a[NR]=$1} END{
    if (NR==0) { print "n=0 min=- med=- max=-"; exit }
    m=(NR%2)?a[(NR+1)/2]:(a[NR/2]+a[NR/2+1])/2
    printf "n=%d min=%.2f med=%.2f max=%.2f\n", NR, a[1], m, a[NR] }'
}

diagnostics() {
  echo "container user: $(id)"
  echo "container groups: $(getent group video render | tr '\n' ' ')"
  echo '$ ls -ln /dev/dri'; ls -ln /dev/dri 2>&1
  for n in /dev/dri/card* /dev/dri/renderD*; do
    [ -e "$n" ] || continue
    if [ -r "$n" ] && [ -w "$n" ]; then echo "$n: usable by this user"; else echo "$n: NOT usable by this user"; fi
  done
  if [ "$MODE" = nvidia ]; then echo '$ nvidia-smi -L'; nvidia-smi -L 2>&1 | head -3; fi
  sudo apt-get update -qq > /dev/null 2>&1
  sudo apt-get install -y -qq --no-install-recommends mesa-utils > /dev/null 2>&1 || echo "mesa-utils install FAILED"
  echo '$ glxinfo -B'; glxinfo -B 2>&1 | grep -E "$PAT"
}

rtf_run() {   # <kind> <gz sim options...>
  kind=$1; shift
  rm -f /tmp/gz.log
  ( timeout -s INT $((WARMUP + SAMPLES + 25)) gz sim "$@" shapes.sdf > /tmp/gz.log 2>&1 < /dev/null & )
  sleep "$WARMUP"
  topic=$(gz topic -l 2> /dev/null | grep -m1 '/stats$')
  if [ -z "$topic" ]; then
    echo "RESULT mode=$MODE rep=$REP kind=$kind n=0 min=- med=- max=- (no /stats topic: gz did not start)"
  else
    vals=$(timeout $((SAMPLES + 15)) gz topic -e -t "$topic" -n "$SAMPLES" 2> /dev/null | awk '/real_time_factor/ {print $2}')
    echo "RESULT mode=$MODE rep=$REP kind=$kind $(printf '%s\n' "$vals" | stats)"
  fi
  grep -E "\[Err\]|OGRE EXCEPTION|Segmentation" /tmp/gz.log | sed 's/\x1b\[[0-9;]*m//g' | sort -u | head -3
  pkill -f 'gz sim' 2> /dev/null
  sleep 2
}

echo "loadavg before: $(cut -d' ' -f1-3 /proc/loadavg)"
case "$MODE" in
  sw|dri|nvidia) [ "$REP" = 1 ] && diagnostics; rtf_run gui -r ;;
  dri-names|dri-nogroup) diagnostics ;;
  headless) rtf_run headless -s -r ;;
esac
INNER

# ---------------------------------------------------------------- B. container runs
if command -v xhost > /dev/null; then xhost +SI:localuser:"$(id -un)" > /dev/null 2>&1; fi
export USER_UID="$(id -u)" USER_GID="$(id -g)"
if [ "$USER_UID" = 0 ]; then USER_UID=1000; USER_GID=1000; fi
: > "$TMP/results.log"

run_mode() {   # <mode> <rep> <overlay file or ""> [extra docker compose run options...]
  local mode=$1 rep=$2 overlay=$3; shift 3
  local files=(-f compose.yaml)
  [ -n "$overlay" ] && files+=(-f "$overlay")
  echo
  echo "----- mode=$mode rep=$rep -----" | tee -a "$TMP/results.log"
  docker compose "${files[@]}" run --rm -T -e MODE="$mode" -e REP="$rep" \
    -e WARMUP="$WARMUP" -e SAMPLES="$SAMPLES" -v "$TMP/inner.sh:/tmp/gpu-test-inner.sh:ro" \
    "$@" dev bash /tmp/gpu-test-inner.sh < /dev/null 2>&1 \
    | tee -a "$TMP/results.log"
}

sec "B. Container runs"
for rep in $(seq 1 "$REPS"); do
  modes=(sw)
  [ "$HAVE_DRI" = 1 ] && modes+=(dri)
  [ "$HAVE_NVIDIA" = 1 ] && modes+=(nvidia)
  # Alternate the order on even repetitions, so "first run is slower" can't favour one mode.
  if (( rep % 2 == 0 )); then
    reversed=(); for ((i=${#modes[@]}-1; i>=0; i--)); do reversed+=("${modes[$i]}"); done
    modes=("${reversed[@]}")
  fi
  for m in "${modes[@]}"; do
    case "$m" in
      sw)     run_mode sw "$rep" "" -e LIBGL_ALWAYS_SOFTWARE=1 ;;
      dri)    run_mode dri "$rep" "$TMP/dri.yaml" ;;
      nvidia) run_mode nvidia "$rep" "$TMP/nvidia.yaml" ;;
    esac
  done
done
if [ "$HAVE_DRI" = 1 ]; then
  run_mode dri-names 1 "$TMP/dri-names.yaml"
  run_mode dri-nogroup 1 "$TMP/dri-nogroup.yaml"
fi
run_mode headless 1 ""

# ---------------------------------------------------------------- summary
sec "Summary: median RTF of each run (1.00 = real time)"
grep '^RESULT ' "$TMP/results.log" | awk '{
  for (i=2;i<=NF;i++) { split($i, kv, "="); v[kv[1]]=kv[2] }
  key=(v["kind"]=="gui") ? v["mode"] : "headless"; list[key]=list[key]" "v["med"] }
  END { for (k in list) printf "%-16s%s\n", k, list[k] }' | sort
echo
echo "Renderer seen in each mode (first repetition):"
awk '/^----- mode=/ { mode=$2; rep=$3 } /OpenGL renderer string/ && rep == "rep=1" {
       sub(/^OpenGL renderer string: /, ""); sub(/^mode=/, "", mode); printf "  %-14s %s\n", mode, $0 }' \
  "$TMP/results.log"

sec "Done"
echo "Share the file:"
echo "  git pull && git add $OUT && git commit -m \"chore(gpu): add gpu test for ${NAME}\" && git push"
sleep 1
