#!/usr/bin/env python3
"""Bounded serial capture for bench bring-up.

Opens the HALMET USB-UART at 115200, pulses the ESP32 auto-reset line so we
catch the boot banner, then prints whatever the board emits for a fixed window
and exits. Not part of the firmware -- a bench tool only.
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-110"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 9.0

ser = serial.Serial(port, 115200, timeout=0.2)

# ESP32 classic reset: pull EN low (RTS) briefly with GPIO0 high (DTR de-asserted)
# so the chip resets and runs the app normally (not the ROM bootloader).
ser.dtr = False
ser.rts = True
time.sleep(0.1)
ser.rts = False

deadline = time.time() + seconds
while time.time() < deadline:
    chunk = ser.read(4096)
    if chunk:
        sys.stdout.buffer.write(chunk)
        sys.stdout.buffer.flush()
ser.close()
