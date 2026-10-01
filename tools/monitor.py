#!/usr/bin/env python3
"""Read the camera's USB serial. Optionally reboot it first and catch the boot lines.

  tools/monitor.py [seconds] [--reboot] [--grep PATTERN]
"""
import os, re, subprocess, sys, time
import serial

PORT = '/dev/ttyACM0'
HERE = os.path.dirname(os.path.abspath(__file__))
PICOTOOL = os.path.join(HERE, 'picotool')

args = sys.argv[1:]
reboot = '--reboot' in args
pattern = None
if '--grep' in args:
    pattern = args[args.index('--grep') + 1]
secs = next((float(a) for a in args if re.fullmatch(r'[\d.]+', a)), 15.0)

if reboot:
    subprocess.run(['sudo', PICOTOOL, 'reboot', '-f'], capture_output=True)
    t = time.time()
    while time.time() - t < 10 and os.path.exists(PORT):
        time.sleep(0.05)
    while time.time() - t < 20 and not os.path.exists(PORT):
        time.sleep(0.05)
    time.sleep(0.6)

ser = None
for _ in range(8):                      # the port can take a moment to settle
    try:
        ser = serial.Serial(PORT, 115200, timeout=2)
        break
    except Exception:
        time.sleep(0.7)
if ser is None:
    sys.exit(f'could not open {PORT}')

start = time.time()
while time.time() - start < secs:
    line = ser.readline().decode(errors='replace').rstrip()
    if not line:
        continue
    if pattern and not re.search(pattern, line):
        continue
    print(f'[{time.time()-start:6.2f}] {line}')
    sys.stdout.flush()
