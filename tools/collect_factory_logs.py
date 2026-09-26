#!/usr/bin/env python3
"""Collect a programmer board's factory log into a master CSV.

Plug a probe (pico_rvd_factory or pico_rvd build) into USB, then:

    collect_factory_logs.py /dev/cu.usbmodemXXXX1 [--master factory_master.csv] [--clear]

Runs `factory log` on the probe's console, appends any records not already
present (keyed on probe serial + sequence number) to the master CSV, and
stamps each with the collection time -- the probes have no clock of their
own. With --clear, the probe's log is erased after a successful collection.

Repeat for each programmer board; the master file accumulates them all.
"""

import argparse
import csv
import datetime
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial required: pip install pyserial")

FIELDS = ["collected_at", "probe", "seq", "uid", "firmware",
          "status", "attempts", "ms", "uptime_s"]

# Anything but the magic baud rates: 1200 drops the probe into BOOTSEL and
# 2400 restarts the firmware.
BAUD = 115200


def read_until_quiet(port, quiet_s=1.5, total_s=30.0):
    out = b""
    last = time.time()
    start = last
    while time.time() - start < total_s:
        chunk = port.read(4096)
        if chunk:
            out += chunk
            last = time.time()
        elif time.time() - last > quiet_s:
            break
    return out.decode(errors="replace")


def command(port, cmd, **kwargs):
    port.reset_input_buffer()
    port.write((cmd + "\n").encode())
    return read_until_quiet(port, **kwargs)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="probe console device (first of its two CDC ports)")
    ap.add_argument("--master", default="factory_master.csv")
    ap.add_argument("--clear", action="store_true",
                    help="erase the probe's log after a successful collection")
    args = ap.parse_args()

    seen = set()
    if os.path.exists(args.master):
        with open(args.master, newline="") as f:
            for row in csv.DictReader(f):
                seen.add((row["probe"], row["seq"]))

    port = serial.Serial(args.port, BAUD, timeout=0.5)
    text = command(port, "factory log")

    records = []
    probe = None
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("CSV,"):
            if line.startswith("#"):
                print(line)
            continue
        parts = line.split(",")
        if len(parts) != 9 or parts[1] == "probe":  # header line echoes the format
            continue
        _, probe, seq, uid, firmware, status, attempts, ms, uptime_s = parts
        records.append({
            "collected_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "probe": probe, "seq": seq, "uid": uid, "firmware": firmware,
            "status": status, "attempts": attempts, "ms": ms, "uptime_s": uptime_s,
        })

    if not records:
        sys.exit("no records found -- is this the console port, and is the "
                 "probe running a build with a factory log?")

    fresh = [r for r in records if (r["probe"], r["seq"]) not in seen]
    new_file = not os.path.exists(args.master)
    with open(args.master, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new_file:
            w.writeheader()
        w.writerows(fresh)

    ok = sum(1 for r in records if r["status"] == "ok")
    print(f"probe {probe}: {len(records)} records ({ok} ok), "
          f"{len(fresh)} new -> {args.master}")

    if args.clear:
        command(port, "factory stop", total_s=10)
        text = command(port, "factory clear", total_s=60)
        if "cleared" in text:
            print("probe log cleared")
        else:
            sys.exit(f"clear did not confirm:\n{text}")


if __name__ == "__main__":
    main()
