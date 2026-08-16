#!/usr/bin/env python3
"""Grab a JPEG from Freddie 2 over the USB console.

Sends the `j` command and decodes the base64 reply into a .jpg.

Usage: python3 tools/grab_frame.py [port] [outfile]
Defaults: /dev/ttyACM0, frame.jpg

Notes:
- Close `idf.py monitor` first; only one process can own the port.
- Needs pyserial, which the ESP-IDF environment provides — run
  `source ~/esp/esp-idf/export.sh` first if the import fails.
"""

import base64
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
out = sys.argv[2] if len(sys.argv) > 2 else "frame.jpg"

s = serial.Serial(port, 115200, timeout=2)
s.reset_input_buffer()
s.write(b"j\n")

data = []
capture = False
deadline = time.time() + 20
while time.time() < deadline:
    line = s.readline().decode("ascii", "replace").strip()
    if line.startswith("JPEG-BEGIN"):
        capture = True
    elif line.startswith("JPEG-END"):
        raw = base64.b64decode("".join(data))
        with open(out, "wb") as f:
            f.write(raw)
        print(f"{out}: {len(raw)} bytes")
        sys.exit(0)
    elif capture and line:
        data.append(line)

print("timed out — is the firmware running? is idf.py monitor closed?")
sys.exit(1)
