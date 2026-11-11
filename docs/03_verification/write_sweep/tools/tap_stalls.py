#!/usr/bin/env python3
"""
Count stalls in a tap of the serial link: get_angles requests that got no reply.

    python3 tap_stalls.py TAP.csv [TAP2.csv ...]

TAP.csv is written by proxy_tap.py: one row per chunk, "seconds,DRV|ATOM,bytes,hex". DRV is
what the driver wrote to the arm, ATOM what the arm sent back.

For every get_angles request (fe fe 02 20 fa) the script finds the next complete reply
(fe fe 0e 20, six int16 angles, fa) and measures the round trip. A request with no reply before
the next request is a stall. Replies that repeat the previous angles are counted too: the arm
answered, only with the position it already had. Those are not stalls.
"""

import argparse
import csv
import sys
from dataclasses import dataclass

GET_REQUEST = bytes.fromhex('fefe0220fa')
SEND_HEADER = bytes.fromhex('fefe0f22')
REPLY_HEADER = bytes.fromhex('fefe0e20')
REPLY_LEN = 17  # header (4), six int16 angles (12), end byte (1)
REPLY_END = 0xFA
ANGLES_AT = 4  # offset of the first angle inside a reply
JOINTS = 6


@dataclass(frozen=True)
class Numbers:
    """What one tap says."""

    requests: int
    sends: int
    replies: int
    stalls: list  # seconds of every request that got no reply
    repeated: int  # replies equal to the previous one
    rtt_ms: list  # round trip of every answered request, sorted


def load(path):
    """Return the rows of a tap as (seconds, who, bytes)."""
    with open(path, newline='', encoding='utf-8') as f:
        return [(float(t), who, bytes.fromhex(hexs)) for t, who, _, hexs in csv.reader(f)]


def decode_reply(stream, start):
    """Return the six angles of the reply that begins at `start`, as the firmware sends them."""
    return [
        int.from_bytes(stream[start + ANGLES_AT + 2 * k:start + ANGLES_AT + 2 * k + 2],
                       'big', signed=True)
        for k in range(JOINTS)
    ]


def replies(rows):
    """Return (end time, angles) of every complete get_angles reply in the ATOM stream."""
    # A reply arrives in several chunks, so the chunks are joined into one stream first, with
    # the time of every byte kept alongside: the reply's time is that of its last byte.
    atom = [(t, b) for t, who, data in rows if who == 'ATOM' for b in data]
    stream = bytes(b for _, b in atom)
    times = [t for t, _ in atom]
    out = []
    start = stream.find(REPLY_HEADER)
    while 0 <= start <= len(stream) - REPLY_LEN:
        end = start + REPLY_LEN - 1
        if stream[end] == REPLY_END:
            out.append((times[end], decode_reply(stream, start)))
        start = stream.find(REPLY_HEADER, start + 1)
    return out


def match(requests, reply_times):
    """
    Pair every request with its reply. Return (round trips in ms, seconds of the stalls).

    A reply belongs to a request when it ended after that request and before the next one.
    A request with no such reply is a stall. Both lists are in time order, so one pass with a
    moving index over the replies is enough.
    """
    rtt_ms, stalls = [], []
    r = 0
    for k, request in enumerate(requests):
        next_request = requests[k + 1] if k + 1 < len(requests) else float('inf')
        while r < len(reply_times) and reply_times[r] < request:
            r += 1  # a reply from before this request: already used, or noise
        if r < len(reply_times) and reply_times[r] < next_request:
            rtt_ms.append((reply_times[r] - request) * 1000)
            r += 1
        else:
            stalls.append(request)
    return rtt_ms, stalls


def analyse(rows):
    """Return the Numbers of one tap."""
    requests = [t for t, who, d in rows if who == 'DRV' and d[:5] == GET_REQUEST]
    sends = [t for t, who, d in rows if who == 'DRV' and d[:4] == SEND_HEADER]
    got = replies(rows)
    rtt_ms, stalls = match(requests, [t for t, _ in got])
    repeated = sum(1 for a, b in zip(got, got[1:]) if a[1] == b[1])
    return Numbers(len(requests), len(sends), len(got), stalls, repeated, sorted(rtt_ms))


def report(path, n):
    """Return the lines printed for one tap."""
    lines = [f'{path}: {n.requests} requests, {n.replies} replies, {n.sends} sends, '
             f'{len(n.stalls)} stalls, {n.repeated} repeated poses']
    if n.rtt_ms:
        p50 = n.rtt_ms[len(n.rtt_ms) // 2]
        p99 = n.rtt_ms[int(len(n.rtt_ms) * 0.99)]
        lines.append(f'  round trip ms: p50 {p50:.1f}  p99 {p99:.1f}  max {n.rtt_ms[-1]:.1f}')
    if n.stalls:
        lines.append('  stalls at s: ' + ', '.join(f'{t:.1f}' for t in n.stalls[:10]))
    return lines


def main():
    """Print requests, replies, sends, stalls, repeated poses and the round trip of each csv."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('taps', nargs='+', help='csv files written by proxy_tap.py')
    for path in parser.parse_args().taps:
        print('\n'.join(report(path, analyse(load(path)))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
