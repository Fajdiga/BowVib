#!/usr/bin/env python3
"""Send simple debug commands to the BowVib ESP32-C6 over USB Serial/JTAG."""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="USB Serial/JTAG port, for example COM92")
    parser.add_argument("command", choices=("STATUS", "SCAN", "LEDTEST", "CSATEST", "HELP"))
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("Install pyserial with: python -m pip install pyserial", file=sys.stderr)
        return 2

    expected = {
        "STATUS": "STATUS ",
        "SCAN": "SCAN ",
        "LEDTEST": "LEDTEST DONE",
        "CSATEST": "CSATEST DONE",
        "HELP": "CSATEST toggles",
    }[args.command]
    try:
        port = serial.Serial(port=None, baudrate=115200, timeout=0.25)
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        with port:
            time.sleep(0.35)
            port.reset_input_buffer()
            port.write((args.command + "\n").encode("ascii"))
            port.flush()
            deadline = time.monotonic() + (15.0 if args.command in ("LEDTEST", "CSATEST") else 6.0)
            while time.monotonic() < deadline:
                line = port.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                print(line)
                if line.startswith("ERR "):
                    return 1
                if line.startswith(expected):
                    return 0
            raise TimeoutError(f"board did not complete {args.command}")
    except (OSError, serial.SerialException, TimeoutError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
