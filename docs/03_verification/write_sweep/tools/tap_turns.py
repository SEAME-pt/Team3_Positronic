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

Since #170 the pose comes from the Atom's poll and a get_angles after a send is the fallback, so
the script also counts the sends followed by a get_angles before the next send, and the interval
from each send to the next in the stream (sends less than 200 ms apart): p50, p90, p99, max and
the share over --slow-ms. The sync-write end is measured for every send, with or without a
get_angles after it.

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


def stream(sends, gap_s=0.2):
    """Return the stream: its send-to-send intervals (ms), its first and its last send time.

    The stream is the longest run of sends less than `gap_s` apart; the sends outside it are the
    rest before and after it.
    """
    if not sends:
        return [], 0.0, 0.0
    best_start, best_stop = 0, 0
    start = 0
    for index in range(1, len(sends)):
        if sends[index] - sends[index - 1] >= gap_s:
            start = index
        if index - start > best_stop - best_start:
            best_start, best_stop = start, index
    intervals = [(sends[index] - sends[index - 1]) * 1000
                 for index in range(best_start + 1, best_stop + 1)]
    return intervals, sends[best_start], sends[best_stop]


def report(path, late_ms, slow_ms):
    """Print the turn numbers of one tap."""
    atom, atom_times, sends, gets = read_tap(path)
    intervals, stream_start, stream_end = stream(sends)
    in_stream = 0
    send_to_poll = []
    sw_end = []
    poll_after_sw = []
    late = []
    no_sync_write = 0
    for index, sent in enumerate(sends):
        # A sync-write after the next send_angles belongs to that one.
        next_send = float('inf')
        if index + 1 < len(sends):
            next_send = sends[index + 1]
        end = sync_write_end(atom, atom_times, sent, next_send)
        if end is None:
            no_sync_write += 1
        else:
            sw_end.append((end - sent) * 1000)
        # The get_angles of this turn, if one went out before the next send (#170: the fallback).
        next_get = bisect.bisect_right(gets, sent)
        polled = None
        if next_get < len(gets) and gets[next_get] < next_send:
            polled = gets[next_get]
            # The last send of the stream is left out: the get_angles after it is the rest's.
            if stream_start <= sent < stream_end:
                in_stream += 1
                send_to_poll.append((polled - sent) * 1000)
        if end is not None and polled is not None:
            poll_after_sw.append((polled - end) * 1000)
        if end is not None and (end - sent) * 1000 > late_ms:
            poll_ms = float('nan')
            if polled is not None:
                poll_ms = (polled - sent) * 1000
            late.append((sent, (end - sent) * 1000, poll_ms))
    print(path)
    print(f'  sends: {len(sends)}, no sync-write before the next send: {no_sync_write}')
    if intervals:
        seconds = sum(intervals) / 1000
        slow = sum(1 for value in intervals if value > slow_ms)
        print(f'  stream: {len(intervals) + 1} sends in {seconds:.1f} s, '
              f'{(len(intervals) + 1) / seconds:.1f}/s')
        print(f'  send interval  ms: p50 {percentile(intervals, 50):.1f} '
              f'p90 {percentile(intervals, 90):.1f} p99 {percentile(intervals, 99):.1f} '
              f'max {max(intervals):.1f}; over {slow_ms:g} ms: {slow} '
              f'({100 * slow / len(intervals):.1f} %)')
        print(f'  sends in the stream with a get_angles before the next send: {in_stream} '
              f'({100 * in_stream / (len(intervals) + 1):.1f} %)')
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
    parser.add_argument('--slow-ms', type=float, default=40.0,
                        help='count the send intervals in the stream longer than this (ms)')
    args = parser.parse_args()
    for path in args.taps:
        report(path, args.late_ms, args.slow_ms)
    return 0


if __name__ == '__main__':
    sys.exit(main())
