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

ClearCore motor test and person tracking

clearcore_motor_controller.ino replaces the supplied motor sketch while
preserving its pin assignments. Flash it to ClearCore first. Keep the dog raised,
use a physical power cutoff, and start with the low MOTOR_PWM value already in
the sketch. Its calibration procedure assumes the zero sensor behaves exactly as
it did in the supplied sketch: the clockwise span between sensor transitions is
180 degrees.

Connect both USB devices to the Pi and identify them:

python -m serial.tools.list_ports -v

Then run:

python pi_lidar_motor_control.py \
  --lidar-port /dev/ttyUSB0 \
  --motor-port /dev/ttyACM0

Do not assume those device names; use the names shown on your Pi. The two ports
must be different.

Keyboard controls:

Key                     Function

H                     Run the bounded 180-degree calibration/home procedure

E                     Enable position commands after successful homing

X                     Disable motor output and automatic mode

Left / A              Manual 2-degree move left

Right / D             Manual 2-degree move right

C                     Move to the 90-degree center

T                     Toggle automatic closest-human-candidate tracking

Space                 Stop and leave automatic mode

Q                     Stop, disable, and exit

Safe first test order:

Raise and secure the dog; have physical power removal within reach.

Start the Pi program. The motor begins disabled.

Press H and verify calibration stops at the expected boundary.

Press E, use one left/right jog, and verify direction.

Test both 0- and 180-degree software limits and the Space stop.

Kill the Pi process while moving; ClearCore must disable within 0.5 seconds.

Run with motor disabled and observe candidate selection on the radar.

Only after all earlier tests pass, press E and then T for slow tracking.

If the motor moves opposite the displayed person, restart with
--invert-motor. Automatic mode accepts only 0-180 degree LiDAR points farther
than one foot, confirms a candidate over multiple frames, limits each command to
two degrees, and stops after target loss. A 2D LiDAR human label remains a
geometric candidate and can mistake furniture for a person.