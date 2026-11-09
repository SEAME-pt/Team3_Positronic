"""Count stalls in a tap of the serial link: get_angles requests that got no reply.

usage: python3 tap_stalls.py TAP.csv [TAP2.csv ...]

TAP.csv is written by proxy_tap.py: one row per chunk, "seconds,DRV|ATOM,bytes,hex". DRV is
what the driver wrote to the arm, ATOM what the arm sent back.

For every get_angles request (fe fe 02 20 fa) the script finds the next complete reply
(fe fe 0e 20, six int16 angles, fa) and measures the round trip. A request with no reply before the
next request is a stall. Replies that repeat the previous angles are counted too: the arm answered,
only with the position it already had. Those are not stalls.
"""

import csv
import sys

GET_REQUEST = bytes.fromhex("fefe0220fa")
SEND_HEADER = bytes.fromhex("fefe0f22")
REPLY_HEADER = bytes.fromhex("fefe0e20")
REPLY_LEN = 17


def load(path):
    """Return the rows of a tap as (seconds, who, bytes)."""
    with open(path, newline="") as f:
        return [(float(t), who, bytes.fromhex(hexs)) for t, who, _, hexs in csv.reader(f)]


def replies(rows):
    """Return (end time, angles) of every complete get_angles reply in the ATOM stream."""
    atom = [(t, b) for t, who, data in rows if who == "ATOM" for b in data]
    stream = bytes(b for _, b in atom)
    times = [t for t, _ in atom]
    out = []
    i = stream.find(REPLY_HEADER)
    while 0 <= i <= len(stream) - REPLY_LEN:
        if stream[i + REPLY_LEN - 1] == 0xFA:
            angles = [int.from_bytes(stream[i + 4 + 2 * k:i + 6 + 2 * k], "big", signed=True)
                      for k in range(6)]
            out.append((times[i + REPLY_LEN - 1], angles))
        i = stream.find(REPLY_HEADER, i + 1)
    return out


def analyse(rows):
    """Return the numbers of one tap as a dict."""
    gets = [t for t, who, d in rows if who == "DRV" and d[:5] == GET_REQUEST]
    sends = [t for t, who, d in rows if who == "DRV" and d[:4] == SEND_HEADER]
    got = replies(rows)
    rtts, stalls = [], []
    r = 0
    for k, request in enumerate(gets):
        next_request = gets[k + 1] if k + 1 < len(gets) else float("inf")
        while r < len(got) and got[r][0] < request:
            r += 1
        if r < len(got) and got[r][0] < next_request:
            rtts.append((got[r][0] - request) * 1000)
            r += 1
        else:
            stalls.append(request)
    repeated = sum(1 for a, b in zip(got, got[1:]) if a[1] == b[1])
    rtts.sort()
    return {
        "requests": len(gets),
        "sends": len(sends),
        "replies": len(got),
        "stalls": stalls,
        "repeated": repeated,
        "rtt_p50": rtts[len(rtts) // 2] if rtts else None,
        "rtt_p99": rtts[int(len(rtts) * 0.99)] if rtts else None,
        "rtt_max": rtts[-1] if rtts else None,
    }


def main(paths):
    """Print requests, replies, sends, stalls, repeated poses and the round trip of each csv."""
    for path in paths:
        n = analyse(load(path))
        print(f"{path}: {n['requests']} requests, {n['replies']} replies, {n['sends']} sends, "
              f"{len(n['stalls'])} stalls, {n['repeated']} repeated poses")
        if n["rtt_p50"] is not None:
            print(f"  round trip ms: p50 {n['rtt_p50']:.1f}  p99 {n['rtt_p99']:.1f}"
                  f"  max {n['rtt_max']:.1f}")
        if n["stalls"]:
            print("  stalls at s: " + ", ".join(f"{t:.1f}" for t in n["stalls"][:10]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
