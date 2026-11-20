# AI-generated (Claude, Anthropic).
# Prompt:
#   #133 spike, approach "adaptive". Read a proxy_tap.py csv (#32: "seconds,DRV|ATOM,bytes,hex")
#   and say what the Atom does with the speed byte of send_angles. For every send_angles: its
#   time, byte and six targets; the Atom's sync-writes to the servos until the next send (count,
#   and per servo the acceleration, goal, goal time and goal speed of the first and the last);
#   the last pose read before the send. Summaries: sync-writes per send (one in fresh mode 1,
#   one every 10 ms in mode 0), the servo goal speed against the byte and against the distance
#   the servo has to go, and the joint speed seen in the poses. --csv writes one row per send
#   and servo, for plotting.
"""
What the Atom does with the speed byte of send_angles, from a tap of the serial link (#133).

    python3 tap_speed.py TAP.csv [TAP2.csv ...] [--csv rows.csv]

TAP.csv is written by ../../write_sweep/tools/proxy_tap.py. The Atom passes a send_angles on to
the servos with a sync-write (ff ff fe 34 83 29 07, 56 bytes) that writes, per servo, 7 bytes
from address 0x29 of the Feetech STS map: acceleration (1 byte, x100 steps/s^2), goal position
(2, steps, 2048 = 0 deg), goal time (2) and goal speed (2, steps/s), little-endian. 4096 steps
per turn. In fresh mode 0 the Atom interpolates and sync-writes an intermediate goal every
10 ms; in mode 1 once per send (#150).
"""

import argparse
import bisect
import csv
import statistics
import sys

SEND_HEADER = bytes.fromhex('fefe0f22')
SEND_SIZE = 18
REPLY_HEADER = bytes.fromhex('fefe0e20')
REPLY_SIZE = 17
SYNC_WRITE = bytes.fromhex('fffffe34832907')
SYNC_WRITE_SIZE = 56
STEP_DEG = 360.0 / 4096


def int16_be(high, low):
    """Return the signed 16-bit big-endian value of two bytes."""
    value = high << 8 | low
    return value - 65536 if value > 32767 else value


def read_tap(path):
    """Return (driver bytes, their times, atom bytes, their times), one time per byte."""
    drv, drv_t, atom, atom_t = bytearray(), [], bytearray(), []
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) != 4:
                continue  # a last row cut by Ctrl+C
            try:
                seconds, data = float(row[0]), bytes.fromhex(row[3])
            except ValueError:
                continue
            if row[1] == 'DRV':
                drv += data
                drv_t += [seconds] * len(data)
            elif row[1] == 'ATOM':
                atom += data
                atom_t += [seconds] * len(data)
    return bytes(drv), drv_t, bytes(atom), atom_t


def find_all(data, header, size):
    """Yield the start of every frame of `size` bytes that begins with `header`."""
    start = data.find(header)
    while start >= 0 and start + size <= len(data):
        yield start
        start = data.find(header, start + 1)


def sends(drv, drv_t):
    """Return [(time, byte, targets in deg)] of every well-formed send_angles."""
    found = []
    for i in find_all(drv, SEND_HEADER, SEND_SIZE):
        frame = drv[i:i + SEND_SIZE]
        if frame[-1] != 0xFA:
            continue
        targets = [int16_be(frame[4 + 2 * j], frame[5 + 2 * j]) / 100 for j in range(6)]
        found.append((drv_t[i], frame[16], targets))
    return found


def sync_writes(atom, atom_t):
    """Return [(time, [(acc, goal deg, goal time, speed steps/s)] x 6)] with a good checksum."""
    found = []
    for i in find_all(atom, SYNC_WRITE, SYNC_WRITE_SIZE):
        frame = atom[i:i + SYNC_WRITE_SIZE]
        if (~sum(frame[2:55])) & 0xFF != frame[55]:
            continue
        servos = []
        for s in range(6):
            o = 7 + 8 * s
            servos.append((frame[o + 1], (frame[o + 2] | frame[o + 3] << 8) - 2048,
                           frame[o + 4] | frame[o + 5] << 8, frame[o + 6] | frame[o + 7] << 8))
        found.append((atom_t[i], [(a, g * STEP_DEG, t, v) for a, g, t, v in servos]))
    return found


def poses(atom, atom_t):
    """Return [(time, six angles in deg)] of every get_angles reply."""
    found = []
    for i in find_all(atom, REPLY_HEADER, REPLY_SIZE):
        frame = atom[i:i + REPLY_SIZE]
        if frame[-1] != 0xFA:
            continue
        found.append((atom_t[i], [int16_be(frame[4 + 2 * j], frame[5 + 2 * j]) / 100
                                  for j in range(6)]))
    return found


def per_send(path):
    """Return one dict per send: time, byte, targets, pose before, sync-writes until the next."""
    drv, drv_t, atom, atom_t = read_tap(path)
    sent, writes, seen = sends(drv, drv_t), sync_writes(atom, atom_t), poses(atom, atom_t)
    write_times = [t for t, _ in writes]
    pose_times = [t for t, _ in seen]
    rows = []
    for k, (t, byte, targets) in enumerate(sent):
        t_next = sent[k + 1][0] if k + 1 < len(sent) else float('inf')
        lo, hi = bisect.bisect_left(write_times, t), bisect.bisect_left(write_times, t_next)
        p = bisect.bisect_left(pose_times, t) - 1
        rows.append({'time': t, 'byte': byte, 'targets': targets,
                     'pose': seen[p][1] if p >= 0 else None,
                     'writes': writes[lo:hi]})
    return rows, seen


def joint_speeds(seen, window_s=0.1):
    """Return the largest speed of each joint over `window_s`, deg/s, from the poses."""
    best = [0.0] * 6
    j = 0
    for i, (t, angles) in enumerate(seen):
        while seen[j][0] < t - window_s:
            j += 1
        dt = t - seen[j][0]
        if dt >= window_s / 2:
            for n in range(6):
                best[n] = max(best[n], abs(angles[n] - seen[j][1][n]) / dt)
    return best


def report(path, out_rows):
    """Print the summary of one tap and append its rows for --csv."""
    rows, seen = per_send(path)
    print(f'## {path}: {len(rows)} sends, {len(seen)} poses')
    if not rows:
        return
    counts = [len(r['writes']) for r in rows]
    print(f'sync-writes per send: median {statistics.median(counts)}, max {max(counts)}'
          f' ({"mode 0, interpolated" if statistics.median(counts) > 2 else "mode 1, one per send"})')
    bytes_seen = sorted({r['byte'] for r in rows})
    print('bytes sent:', ' '.join(str(b) for b in bytes_seen))
    for byte in bytes_seen:
        mine = [r for r in rows if r['byte'] == byte and r['writes']]
        speeds = [[w[1][s][3] for r in mine for w in r['writes']] for s in range(6)]
        accs = sorted({w[1][s][0] for r in mine for w in r['writes'] for s in range(6)})
        line = ' '.join(f'J{s + 1} {min(v)}-{max(v)}' if v else f'J{s + 1} -'
                        for s, v in enumerate(speeds))
        print(f'  byte {byte:3d}: {len(mine)} sends, servo goal speed steps/s {line}; acc {accs}')
        # How fast the goals themselves move from one sync-write to the next: the Atom's
        # interpolation speed in mode 0 (cruise and ramp), the commanded step in mode 1.
        goal_peak = [0.0] * 6
        for r in mine:
            w = r['writes']
            for a, b in zip(w, w[1:]):
                dt = b[0] - a[0]
                if dt > 0.004:
                    for s in range(6):
                        goal_peak[s] = max(goal_peak[s], abs(b[1][s][1] - a[1][s][1]) / dt)
        print('           peak goal speed between sync-writes, deg/s: '
              + ' '.join(f'J{s + 1} {v:.1f}' for s, v in enumerate(goal_peak)))
        floor = min((min(v) for v in speeds if v), default=None)
        if floor:
            print(f'           lowest goal speed {floor} = {floor / byte:.2f} x byte'
                  f' = {floor * STEP_DEG:.1f} deg/s')
    peak = joint_speeds(seen)
    print('peak joint speed in the poses, deg/s:',
          ' '.join(f'J{n + 1} {v:.1f}' for n, v in enumerate(peak)))
    for r in rows:
        first = r['writes'][0][1] if r['writes'] else [None] * 6
        for s in range(6):
            distance = (abs(r['targets'][s] - r['pose'][s]) if r['pose'] is not None else '')
            out_rows.append([path, f'{r["time"]:.6f}', r['byte'], s + 1, r['targets'][s],
                             r['pose'][s] if r['pose'] is not None else '', distance,
                             len(r['writes'])] + (list(first[s]) if first[s] else ['', '', '', '']))


def main(argv=None):
    """Parse the arguments and report every tap."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('taps', nargs='+')
    parser.add_argument('--csv', help='one row per send and servo')
    args = parser.parse_args(argv)
    out_rows = []
    for path in args.taps:
        report(path, out_rows)
    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['tap', 'time', 'byte', 'joint', 'target_deg', 'pose_deg',
                             'distance_deg', 'sync_writes', 'acc', 'goal_deg', 'goal_time',
                             'goal_speed'])
            writer.writerows(out_rows)
    return 0


if __name__ == '__main__':
    sys.exit(main())
