# LD19 / D300 Python radar

A small, ROS-free and Rust-free live radar for the LDROBOT LD19 (including the
D300 kit). It reads the LiDAR's one-way UART packets, verifies CRC-8, displays
objects, and marks geometrically human-sized clusters as **human candidates**.

## Wiring and serial port

The easiest connection is the D300 USB adapter. It normally appears as
`/dev/ttyUSB0` or `/dev/ttyACM0`.

For direct Raspberry Pi GPIO UART: LiDAR TX (3.3 V) goes to Pi RX, grounds must
be common, and the LiDAR requires a proper 5 V supply. Do not connect LiDAR TX
to a 5 V logic input. Ground the PWM pin if you want the LiDAR's internal speed
control. The stream is **230400 baud, 8 data bits, no parity, 1 stop bit**.

Find the port:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

If permission is denied, add your account to `dialout`, then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

Do not run the graphical program with `sudo`; that often breaks the display and
uses a different Python environment.

## Install and run

```bash
cd ld19-python-radar
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python ld19_radar.py --port /dev/ttyUSB0
```

Useful tuning example:

```bash
python ld19_radar.py --port /dev/ttyUSB0 --max-range 6 \
  --cluster-gap 0.15 --human-min-width 0.10 --human-max-width 0.65
```

Use `--flip` if left and right appear reversed. Press Ctrl+C in the terminal or
close the plot window to stop.

## What "human candidate" means

One horizontal 2D scan cannot uniquely identify a person. This program clusters
neighboring returns and colors clusters red when their visible width falls in a
configurable human/leg-like range. Chairs, posts, and other objects can produce
false positives. For dependable human recognition, fuse this with a camera,
thermal sensor, or multi-frame leg tracker.

Run `python ld19_radar.py --help` to see every threshold.
