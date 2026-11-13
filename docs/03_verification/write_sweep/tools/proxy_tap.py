#!/usr/bin/env python3
"""
Proxy between the driver and the arm that writes every chunk of both directions to a csv.

    python3 proxy_tap.py [TAP.csv]

The driver and the arm talk through one serial port, and only one process can hold it. So the
proxy takes the real port and offers the driver a pseudo-terminal instead: a file that behaves
like a serial port with no cable behind it. Whatever the driver writes there is copied to the
arm, whatever the arm answers is copied back, and every chunk goes to TAP.csv with the moment
it passed: "seconds,DRV|ATOM,bytes,hex". DRV is what the driver wrote, ATOM what the arm sent
(Atom is the board of the myCobot that speaks the serial protocol).

The name of the pseudo-terminal is written to /tmp/proxy_port, so a launch can read it:
PORT=$(cat /tmp/proxy_port). Start the proxy before the driver, stop it with Ctrl+C after the
run. tap_stalls.py reads the csv.
"""

import argparse
import os
import pty
import select
import sys
import time

import serial

ARM_PORT = '/dev/ttyTHS1'
BAUD = 1000000
PORT_NAME_FILE = '/tmp/proxy_port'
DEFAULT_TAP = '/ws/log/tap.csv'
CHUNK = 4096
WAIT_S = 0.5  # how long select() waits before looping again, so Ctrl+C is seen


def open_fake_port():
    """Create the pseudo-terminal, publish the driver's end, and return our end."""
    ours, theirs = pty.openpty()
    name = os.ttyname(theirs)
    with open(PORT_NAME_FILE, 'w', encoding='utf-8') as f:
        f.write(name)
    print('port', name, flush=True)
    return ours


def forward_from_arm(arm, fake, tap, now):
    """Copy what the arm sent to the driver's port and log it as ATOM."""
    data = arm.read(CHUNK)
    if data:
        os.write(fake, data)
        tap.write(f'{now:.6f},ATOM,{len(data)},{data.hex()}\n')


def forward_from_driver(arm, fake, tap, now):
    """Copy what the driver wrote to the arm and log it as DRV. False once the driver is gone."""
    try:
        data = os.read(fake, CHUNK)
    except OSError:
        return False  # the driver closed its end of the pseudo-terminal
    if data:
        arm.write(data)
        tap.write(f'{now:.6f},DRV,{len(data)},{data.hex()}\n')
    return True


def run(arm, fake, tap):
    """Pass bytes both ways and log them until the driver leaves or Ctrl+C."""
    start = time.time()
    while True:
        ready, _, _ = select.select([fake, arm.fileno()], [], [], WAIT_S)
        now = time.time() - start
        if arm.fileno() in ready:
            forward_from_arm(arm, fake, tap, now)
        if fake in ready:
            if not forward_from_driver(arm, fake, tap, now):
                return
        tap.flush()


def main():
    """Parse the csv path, open both ports and the csv, and run until Ctrl+C."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('tap', nargs='?', default=DEFAULT_TAP, help='csv to write')
    args = parser.parse_args()

    # timeout=0: read() returns what is there and never waits. The waiting is select()'s job,
    # on both ends at once, so neither side holds up the other.
    with serial.Serial(ARM_PORT, BAUD, timeout=0) as arm, \
            open(args.tap, 'w', encoding='utf-8') as tap:
        fake = open_fake_port()
        try:
            run(arm, fake, tap)
        except KeyboardInterrupt:
            pass
        finally:
            os.close(fake)
    return 0


if __name__ == '__main__':
    sys.exit(main())
