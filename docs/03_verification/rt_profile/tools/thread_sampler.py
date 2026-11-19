#!/usr/bin/env python3
# Copyright 2026 Positronic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""#124: samples every thread of one process: state, wchan, cpu time, wait time.

    python3 thread_sampler.py --pid <pid> --out threads.csv --duration 600

One row per thread per sample, every 17 ms by default: wall_s, tid, comm, state
(R running, S sleeping, D disk wait), wchan (the kernel function the thread waits in),
utime+stime ticks, schedstat run ns and wait ns. During a pause of the control loop this says
whether a thread was waiting on a futex (a mutex in the process), on I/O, or was runnable but
not scheduled (wait ns growing). Run it as the same user as the process. It stops at the
duration, when the process ends, or on Ctrl+C or SIGTERM, and the csv is complete in every case.
"""
import argparse
import csv
import os
import signal
import time

NAMES = ('stat', 'schedstat', 'wchan')
GONE = (FileNotFoundError, ProcessLookupError)


def drop(fds, tid):
    """Close the files of a thread that ended."""
    for fd in fds.pop(tid, ()):
        os.close(fd)


def open_files(base, tid, files):
    """Open the proc files of a thread that are not open yet."""
    for name in NAMES[len(files):]:
        files.append(os.open(f'{base}/{tid}/{name}', os.O_RDONLY))


def read_all(files):
    """Read the open proc files of one thread from the start."""
    return [os.pread(fd, 1024, 0).decode(errors='replace') for fd in files]


def parse_stat(stat):
    """Return comm, state and utime+stime ticks from a /proc/<pid>/task/<tid>/stat line."""
    end = stat.rfind(')')  # comm may hold spaces or parentheses, so the last ')' closes it
    comm = stat[stat.find('(') + 1:end]
    fields = stat[end + 2:].split()  # fields[0] is the state, field 3 of stat
    cpu_ticks = int(fields[11]) + int(fields[12])  # utime, stime: fields 14 and 15 of stat
    return comm, fields[0], cpu_ticks


def parse_schedstat(sched):
    """Return run ns and wait ns, or empty strings when the kernel does not provide them."""
    parts = sched.split()
    if len(parts) < 2:
        return '', ''
    return parts[0], parts[1]


def sample(base, fds, rows, now):
    """Write one row per thread of the process, keep its proc files open in fds."""
    tids = os.listdir(base)
    for tid in set(fds).difference(tids):
        drop(fds, tid)
    for tid in tids:
        files = fds.setdefault(tid, [])
        try:
            open_files(base, tid, files)
            stat, sched, wchan = read_all(files)
        except GONE:
            drop(fds, tid)
            continue
        comm, state, cpu_ticks = parse_stat(stat)
        run_ns, wait_ns = parse_schedstat(sched)
        rows.writerow((f'{now:.6f}', tid, comm, state, wchan, cpu_ticks, run_ns, wait_ns))


def main():
    """Sample the process until the duration ends, one csv row per thread per sample."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--duration', type=float, default=600.0)
    parser.add_argument('--period', type=float, default=0.017)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    base = f'/proc/{args.pid}/task'
    fds = {}
    end = time.monotonic() + args.duration
    with open(args.out, 'w', newline='') as out:
        rows = csv.writer(out, lineterminator='\n')
        rows.writerow('wall_s,tid,comm,state,wchan,cpu_ticks,run_ns,wait_ns'.split(','))
        try:
            while time.monotonic() < end:
                started = time.monotonic()
                sample(base, fds, rows, time.time())
                spent = time.monotonic() - started
                time.sleep(max(0.0, args.period - spent))
        except GONE:
            pass  # the process ended
        except KeyboardInterrupt:
            pass  # Ctrl+C or kill, the csv is still flushed


if __name__ == '__main__':
    main()
