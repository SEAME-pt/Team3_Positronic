# AI-generated (Claude, Anthropic).
# Prompt:
#   #133 spike, approach "static", the arm run over Felipe's full range of speed bytes (5, 10,
#   30, 50, 70, 90, 100). High bytes step up only: a byte above 30 runs only after the stop
#   distance at the byte before it was measured in the same tip setup. Read the mode folders the
#   arm run left (arm_run_static.sh: summary.txt with tap_speed.py's stop lines, stop_r<n>_
#   trigger.txt, args.txt) and say whether the next byte may start, and why not. A dry run on
#   the fake backend (no tap) never counts for the real arm.
"""
May the arm run start a speed byte? The step-up rule of #133 (Felipe, 2026-11-24).

    python3 step_up_check.py SETUP_DIR BYTE [--max-travel-deg 4] [--dry]

Bytes: 5, 10, 30, 50, 70, 90, 100. Bytes up to 30 (the driver's default) have no condition.
Above 30 each byte needs the one before it on the ladder 30 -> 50 -> 70 -> 90 -> 100 done in
SETUP_DIR: a folder fixed<before> (or a re-run fixed<before>-HHMMSS) with
  - stop_r1..stop_r5_trigger.txt, each "ok=True" (the driver took every stop), and
  - at least 5 "stop at" lines in summary.txt (tap_speed.py), every travel at most
    --max-travel-deg (default 4: about twice the 1.7-2.0 deg measured at 30 and 50).
A folder whose args.txt says DRY (fake backend, no tap) counts only with --dry, and then
without the travel. Exit 0 and the evidence on stdout when the byte may start, 1 and the reason
when it may not, 2 for a byte that is not on the list.
"""

import argparse
import math
import pathlib
import re
import sys

BYTES = (5, 10, 30, 50, 70, 90, 100)
BEFORE = {50: 30, 70: 50, 90: 70, 100: 90}
STOPS = 5
STOP_LINE = re.compile(r'^stop at .*travel after (\S+) deg', re.MULTILINE)


def mode_dirs(setup_dir, byte):
    """The folders of fixed<byte> in setup_dir: the first run and any -HHMMSS re-run."""
    pattern = re.compile(rf'^fixed{byte}(-\d{{6}})?$')
    root = pathlib.Path(setup_dir)
    if not root.is_dir():
        return []
    return sorted(p for p in root.iterdir() if p.is_dir() and pattern.match(p.name))


def is_dry(folder):
    """True when the folder was a dry run on the fake backend."""
    args = folder / 'args.txt'
    return args.is_file() and args.read_text(encoding='utf-8').startswith('driver: DRY')


def triggers_ok(folder):
    """The stop runs whose trigger file says the driver took the stop."""
    runs = set()
    for run in range(1, STOPS + 1):
        path = folder / f'stop_r{run}_trigger.txt'
        if path.is_file() and 'ok=True' in path.read_text(encoding='utf-8'):
            runs.add(run)
    return runs


def stop_travels(folder):
    """The travel after each stop, deg, from tap_speed.py's lines in summary.txt."""
    path = folder / 'summary.txt'
    if not path.is_file():
        return []
    travels = []
    for text in STOP_LINE.findall(path.read_text(encoding='utf-8')):
        try:
            travels.append(float(text))
        except ValueError:
            travels.append(math.nan)
    return travels


def judge(folder, max_travel_deg, dry):
    """(True, evidence) if this folder shows the stops the next byte needs, else (False, why)."""
    if is_dry(folder) and not dry:
        return False, f'{folder.name}: a dry run on the fake backend, not the arm'
    runs = triggers_ok(folder)
    missing = sorted(set(range(1, STOPS + 1)) - runs)
    if missing:
        return False, f'{folder.name}: stops {missing} missing or not taken by the driver'
    if is_dry(folder):
        return True, f'{folder.name}: DRY, {STOPS} stops taken, travel not checked (no tap)'
    travels = stop_travels(folder)
    if len(travels) < STOPS:
        return False, f'{folder.name}: {len(travels)} stops in summary.txt, need {STOPS}'
    worst = max(travels, key=lambda t: math.inf if math.isnan(t) else t)
    if not worst <= max_travel_deg:
        return False, (f'{folder.name}: a stop travelled {worst} deg, '
                       f'more than {max_travel_deg} deg')
    return True, (f'{folder.name}: {len(travels)} stops, travel '
                  f'{min(travels):.2f}-{worst:.2f} deg <= {max_travel_deg} deg')


def check(setup_dir, byte, max_travel_deg=4.0, dry=False):
    """(code, message): 0 the byte may start, 1 it may not, 2 not a byte of the list."""
    if byte not in BYTES:
        return 2, f'byte {byte}: only {", ".join(map(str, BYTES))}'
    if byte not in BEFORE:
        return 0, f'byte {byte}: no condition (30 or below)'
    before = BEFORE[byte]
    folders = mode_dirs(setup_dir, before)
    if not folders:
        return 1, (f'byte {byte}: run fixed{before} with its 5 stops in {setup_dir} first '
                   '(step up only)')
    reasons = []
    for folder in folders:
        ok, why = judge(folder, max_travel_deg, dry)
        if ok:
            return 0, f'byte {byte} may start after fixed{before}: {why}'
        reasons.append(why)
    return 1, f'byte {byte}: fixed{before} is not done: ' + '; '.join(reasons)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('setup_dir')
    parser.add_argument('byte', type=int)
    parser.add_argument('--max-travel-deg', type=float, default=4.0)
    parser.add_argument('--dry', action='store_true')
    args = parser.parse_args(argv)
    code, message = check(args.setup_dir, args.byte, args.max_travel_deg, args.dry)
    print(message, file=sys.stdout if code == 0 else sys.stderr)
    return code


if __name__ == '__main__':
    sys.exit(main())
