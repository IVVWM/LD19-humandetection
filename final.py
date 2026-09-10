#!/usr/bin/env python3
"""Keyboard test and guarded LD19-to-ClearCore person tracking."""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

PACKET_SIZE = 47
FRONT_MIN_DEG = 0.0
FRONT_MAX_DEG = 180.0
FRONT_CENTER_DEG = 90.0
ONE_FOOT_M = 0.3048


def make_crc_table(poly=0x4D):
    table = []
    for value in range(256):
        crc = value
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


CRC_TABLE = make_crc_table()


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = CRC_TABLE[crc ^ byte]
    return crc


@dataclass(frozen=True)
class Point:
    angle: float
    distance: float
    intensity: int


class LD19:
    def __init__(self, port: str):
        self.serial = serial.Serial(port, 230400, timeout=0)
        self.buffer = bytearray()

    def read(self) -> list[Point]:
        self.buffer.extend(self.serial.read(self.serial.in_waiting or 1))
        result = []
        while len(self.buffer) >= PACKET_SIZE:
            try:
                start = self.buffer.index(0x54)
            except ValueError:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < PACKET_SIZE:
                break
            if self.buffer[1] != 0x2C:
                del self.buffer[0]
                continue
            packet = bytes(self.buffer[:PACKET_SIZE])
            if crc8(packet[:-1]) != packet[-1]:
                del self.buffer[0]
                continue
            del self.buffer[:PACKET_SIZE]
            start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
            end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0
            span = (end_angle - start_angle) % 360.0
            for i in range(12):
                distance_mm = struct.unpack_from("<H", packet, 6 + i * 3)[0]
                intensity = packet[8 + i * 3]
                angle = (start_angle + span * i / 11.0) % 360.0
                if distance_mm:
                    result.append(Point(angle, distance_mm / 1000.0, intensity))
        return result

    def close(self):
        self.serial.close()


class ClearCore:
    def __init__(self, port: str):
        self.serial = serial.Serial(port, 115200, timeout=0)
        self.last_heartbeat = 0.0
        time.sleep(2.0)  # USB serial reset/settle time
        self.serial.reset_input_buffer()

    def command(self, text: str):
        self.serial.write((text.strip() + "\n").encode("ascii"))

    def heartbeat(self):
        now = time.monotonic()
        if now - self.last_heartbeat >= 0.1:
            self.command("HEARTBEAT")
            self.last_heartbeat = now

    def responses(self) -> list[str]:
        output = []
        while self.serial.in_waiting:
            line = self.serial.readline().decode("ascii", errors="replace").strip()
            if line and line != "OK HEARTBEAT":
                output.append(line)
        return output

    def close(self):
        try:
            self.command("STOP")
            self.command("DISABLE")
        finally:
            self.serial.close()


def xy(point: Point):
    angle = math.radians(point.angle)
    return point.distance * math.cos(angle), point.distance * math.sin(angle)


def clusters(points: list[Point], maximum_gap=0.18, minimum_points=3):
    points = sorted(points, key=lambda p: p.angle)
    if not points:
        return []
    groups = [[points[0]]]
    for point in points[1:]:
        x1, y1 = xy(groups[-1][-1])
        x2, y2 = xy(point)
        if math.hypot(x2 - x1, y2 - y1) <= maximum_gap:
            groups[-1].append(point)
        else:
            groups.append([point])
    return [group for group in groups if len(group) >= minimum_points]


def width(group: list[Point]):
    coords = np.array([xy(point) for point in group])
    return float(np.linalg.norm(coords.max(axis=0) - coords.min(axis=0)))


def closest_person(points: list[Point]):
    candidates = []
    for group in clusters(points):
        group_width = width(group)
        if 0.08 <= group_width <= 0.80:
            angle = float(np.median([p.angle for p in group]))
            distance = float(np.median([p.distance for p in group]))
            candidates.append((distance, angle, group))
    return min(candidates, default=None, key=lambda item: item[0])


def args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar-port", default="/dev/ttyUSB0")
    parser.add_argument("--motor-port", default="/dev/ttyACM0")
    parser.add_argument("--max-range", type=float, default=8.0)
    parser.add_argument("--invert-motor", action="store_true")
    parser.add_argument("--confirm-frames", type=int, default=4)
    parser.add_argument("--target-timeout", type=float, default=0.75)
    parser.add_argument("--max-command-step", type=float, default=2.0)
    return parser.parse_args()


def main():
    options = args()
    lidar = LD19(options.lidar_port)
    motor = ClearCore(options.motor_port)

    # Each bin contains (point, timestamp); old data expires instead of persisting.
    bins: list[tuple[Point, float] | None] = [None] * 720
    automatic = False
    motor_enabled = False
    commanded_angle = FRONT_CENTER_DEG
    confirmed_frames = 0
    last_candidate_angle = None
    last_confirmed_target = 0.0
    status_message = "Press H to calibrate/home"

    fig, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(9, 6))
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_ylim(0, options.max_range)
    ax.set_title("LD19 motor tracking test — MANUAL")
    objects_plot = ax.scatter([], [], s=10, color="#42a5f5", label="object")
    person_plot = ax.scatter([], [], s=28, color="#ff5252", label="human candidate")
    target_line, = ax.plot([], [], color="#ffeb3b", linewidth=2, label="selected target")
    status = ax.text(0.02, 0.02, status_message, transform=ax.transAxes)
    ax.legend(loc="upper right")

    def send_move(new_angle: float):
        nonlocal commanded_angle
        new_angle = max(0.0, min(180.0, new_angle))
        motor.command(f"MOVE {new_angle:.1f}")
        commanded_angle = new_angle

    def on_key(event):
        nonlocal automatic, motor_enabled, status_message
        if event.key in ("left", "a"):
            automatic = False
            send_move(commanded_angle - 2.0)
        elif event.key in ("right", "d"):
            automatic = False
            send_move(commanded_angle + 2.0)
        elif event.key == "c":
            automatic = False
            send_move(90.0)
        elif event.key == "h":
            automatic = False
            motor.command("HOME")
            status_message = "Calibrating: keep emergency stop ready"
        elif event.key == "e":
            motor.command("ENABLE")
            motor_enabled = True
        elif event.key == "x":
            automatic = False
            motor.command("DISABLE")
            motor_enabled = False
        elif event.key == " ":
            automatic = False
            motor.command("STOP")
        elif event.key == "t":
            automatic = not automatic
        elif event.key == "q":
            plt.close(fig)
        ax.set_title(f"LD19 motor tracking test — {'AUTO' if automatic else 'MANUAL'}")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        nonlocal confirmed_frames, last_candidate_angle, last_confirmed_target
        nonlocal status_message
        now = time.monotonic()
        motor.heartbeat()
        for response in motor.responses():
            print(response)
            status_message = response

        for point in lidar.read():
            allowed = (FRONT_MIN_DEG <= point.angle <= FRONT_MAX_DEG and
                       ONE_FOOT_M <= point.distance <= options.max_range and
                       point.intensity >= 5)
            if allowed:
                bins[round(point.angle * 2) % len(bins)] = (point, now)

        # Only use points refreshed during the last 0.25 seconds.
        visible = [entry[0] for entry in bins if entry and now - entry[1] <= 0.25]
        selected = closest_person(visible)
        selected_group = []

        if selected:
            distance, angle, selected_group = selected
            if last_candidate_angle is not None and abs(angle - last_candidate_angle) <= 8.0:
                confirmed_frames += 1
            else:
                confirmed_frames = 1
            last_candidate_angle = angle

            if confirmed_frames >= options.confirm_frames:
                last_confirmed_target = now
                desired = 180.0 - angle if options.invert_motor else angle
                error = desired - commanded_angle
                if automatic and motor_enabled and abs(error) > 3.0:
                    step = max(-options.max_command_step,
                               min(options.max_command_step, error))
                    send_move(commanded_angle + step)
                status_message = (f"target angle={angle:.1f} deg distance={distance:.2f} m "
                                  f"confirmed={confirmed_frames}")
        else:
            confirmed_frames = 0
            last_candidate_angle = None

        if automatic and now - last_confirmed_target > options.target_timeout:
            motor.command("STOP")

        person_ids = {id(point) for point in selected_group}
        ordinary = [point for point in visible if id(point) not in person_ids]
        people = [point for point in visible if id(point) in person_ids]

        def offsets(points):
            return (np.array([(math.radians(p.angle), p.distance) for p in points])
                    if points else np.empty((0, 2)))

        objects_plot.set_offsets(offsets(ordinary))
        person_plot.set_offsets(offsets(people))
        if selected:
            target_line.set_data([math.radians(selected[1])] * 2, [ONE_FOOT_M, selected[0]])
        else:
            target_line.set_data([], [])
        status.set_text(
            f"{status_message}\nmode={'AUTO' if automatic else 'MANUAL'} "
            f"motor={'ENABLED' if motor_enabled else 'DISABLED'} command={commanded_angle:.1f}°\n"
            "keys: H home | E enable | X disable | arrows/A,D jog | C center | T auto | Space stop | Q quit"
        )
        return objects_plot, person_plot, target_line, status

    animation = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    try:
        plt.show()
    finally:
        _ = animation
        motor.close()
        lidar.close()


if __name__ == "__main__":
    main()
