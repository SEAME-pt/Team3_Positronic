#!/usr/bin/env python3
"""
Time the serial turn after every send_angles in a tap of the serial link (#150).

    python3 tap_turns.py TAP.csv [TAP2.csv ...] [--late-ms 30]

TAP.csv is written by proxy_tap.py: one row per chunk, "seconds,DRV|ATOM,bytes,hex". DRV is
what the driver wrote to the arm, ATOM what the arm sent back.

After a send_angles (fe fe 0f 22 ...) the Atom acknowledges it, polls its six servos and passes
the targets on with one sync-write to the servos (ff ff fe 34 83 29 07, 56 bytes), the last
frame it puts on the line. For every send_angles followed by a get_angles (fe fe 02 20 fa) the
script prints, in ms:

    send_to_poll   the next get_angles after the send
    sync_write_end the last byte of the sync-write after the send
    poll_after_sw  the next get_angles after the end of the sync-write

A negative poll_after_sw is a get_angles written while the Atom was still busy, which the arm
drops: #150 makes the driver wait for the sync-write, so it must never happen. Every send whose
sync-write ended later than --late-ms is listed with its time in the tap.

Only a sync-write with a good checksum counts, and only one that starts before the next
send_angles: a sync-write that comes after the next send (after a timeout) is not told apart
from that send's own, so such a send counts as one with no sync-write. A last row cut by Ctrl+C
on proxy_tap.py is skipped.
"""

import argparse
import bisect
import csv
import statistics
import sys

SEND_HEADER = bytes.fromhex('fefe0f22')
GET_REQUEST = bytes.fromhex('fefe0220fa')
SYNC_WRITE = bytes.fromhex('fffffe34832907')
SYNC_WRITE_SIZE = 56  # the start (7), 6 servos x 8 bytes, the checksum (1)


def percentile(values, p):
    """Return the p-th percentile of `values`, the nearest rank."""
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(p / 100 * (len(ordered) - 1))))]


def summary(values):
    """Return one line for a list of ms values."""
    if not values:
        return 'none'
    return (f'n={len(values)} mean {statistics.mean(values):.1f} '
            f'p50 {percentile(values, 50):.1f} p99 {percentile(values, 99):.1f} '
            f'min {min(values):.1f} max {max(values):.1f}')


def read_tap(path):
    """Return the Atom's bytes with the time of each, and the times of the driver's frames."""
    atom = bytearray()
    atom_times = []
    sends = []
    gets = []
    with open(path, newline='', encoding='utf-8') as tap:
        for row in csv.reader(tap):
            if len(row) != 4:
                continue  # a row cut by Ctrl+C on proxy_tap.py
            seconds, who, _, hex_bytes = row
            try:
                data = bytes.fromhex(hex_bytes)
                at = float(seconds)
            except ValueError:
                continue
            if who == 'ATOM':
                atom += data
                atom_times += [at] * len(data)
            elif data.startswith(SEND_HEADER):
                sends.append(at)
            elif data.startswith(GET_REQUEST):
                gets.append(at)
    return bytes(atom), atom_times, sends, gets


def checksum_ok(frame):
    """Return whether a servo-bus frame ends with ~sum of its bytes from the id on."""
    return (~sum(frame[2:-1])) & 0xFF == frame[-1]


def sync_write_end(atom, atom_times, after, before):
    """Return when the first good sync-write between `after` and `before` ends."""
    start = bisect.bisect_left(atom_times, after)
    stop = bisect.bisect_left(atom_times, before)
    found = atom.find(SYNC_WRITE, start, stop)
    while found >= 0 and found + SYNC_WRITE_SIZE <= len(atom):
        if checksum_ok(atom[found:found + SYNC_WRITE_SIZE]):
            return atom_times[found + SYNC_WRITE_SIZE - 1]
        found = atom.find(SYNC_WRITE, found + 1, stop)
    return None


def report(path, late_ms):
    """Print the turn numbers of one tap."""
    atom, atom_times, sends, gets = read_tap(path)
    send_to_poll = []
    sw_end = []
    poll_after_sw = []
    late = []
    no_sync_write = 0
    for index, sent in enumerate(sends):
        next_get = bisect.bisect_right(gets, sent)
        if next_get >= len(gets):
            continue
        polled = gets[next_get]
        send_to_poll.append((polled - sent) * 1000)
        # A sync-write after the next send_angles belongs to that one.
        next_send = float('inf')
        if index + 1 < len(sends):
            next_send = sends[index + 1]
        end = sync_write_end(atom, atom_times, sent, next_send)
        if end is None:
            no_sync_write += 1
            continue
        sw_end.append((end - sent) * 1000)
        poll_after_sw.append((polled - end) * 1000)
        if (end - sent) * 1000 > late_ms:
            late.append((sent, (end - sent) * 1000, (polled - sent) * 1000))
    print(path)
    print(f'  sends with a next get_angles: {len(send_to_poll)}, '
          f'no sync-write before the next send: {no_sync_write}')
    print(f'  send_to_poll   ms: {summary(send_to_poll)}')
    print(f'  sync_write_end ms: {summary(sw_end)}')
    print(f'  poll_after_sw  ms: {summary(poll_after_sw)}')
    early = sum(1 for value in poll_after_sw if value < 0)
    print(f'  get_angles written before the sync-write ended: {early}')
    print(f'  sync-writes ending later than {late_ms:g} ms: {len(late)}')
    for sent, end_ms, poll_ms in late:
        print(f'    at {sent:8.1f} s: sync-write ended {end_ms:.1f} ms, '
              f'get_angles at {poll_ms:.1f} ms')


def main():
    """Parse the arguments and report every tap."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('taps', nargs='+', help='csv files written by proxy_tap.py')
    parser.add_argument('--late-ms', type=float, default=30.0,
                        help='list the sends whose sync-write ended later than this (ms)')
    args = parser.parse_args()
    for path in args.taps:
        report(path, args.late_ms)
    return 0


if __name__ == '__main__':
    sys.exit(main())
