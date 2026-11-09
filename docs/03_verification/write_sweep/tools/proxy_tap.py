"""Proxy between the driver and the arm that writes every chunk of both directions to a csv.

usage: python3 proxy_tap.py [TAP.csv]

Opens the real port, offers a pty to the driver (its name goes to /tmp/proxy_port) and writes
"seconds,DRV|ATOM,bytes,hex" per chunk to TAP.csv (default /ws/log/tap.csv). Start it before
the driver, stop it with Ctrl+C after the run.
"""
import os
import pty
import select
import sys
import time

import serial

arm = serial.Serial("/dev/ttyTHS1", 1000000, timeout=0)
master, slave = pty.openpty()
with open("/tmp/proxy_port", "w") as f:
    f.write(os.ttyname(slave))
print("port", os.ttyname(slave), flush=True)
tap = open(sys.argv[1] if len(sys.argv) > 1 else "/ws/log/tap.csv", "w")
start = time.time()
while True:
    r, _, _ = select.select([master, arm.fileno()], [], [], 0.5)
    now = time.time() - start
    if arm.fileno() in r:
        data = arm.read(4096)
        if data:
            os.write(master, data)
            tap.write("%.6f,ATOM,%d,%s\n" % (now, len(data), data.hex()))
    if master in r:
        try:
            data = os.read(master, 4096)
        except OSError:
            break
        if data:
            arm.write(data)
            tap.write("%.6f,DRV,%d,%s\n" % (now, len(data), data.hex()))
            tap.flush()
