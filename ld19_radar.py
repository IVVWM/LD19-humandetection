#!/usr/bin/env python3
"""Live radar and simple human-candidate detection for LD19/D300 LiDAR."""

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


# LD19 packet settings
PACKET_SIZE = 47
HEADER = 0x54
VER_LEN = 0x2C
SAMPLES_PER_PACKET = 12

# Detection region
ONE_FOOT_METERS = 0.3048
DETECTION_ANGLE_MIN = 0.0
DETECTION_ANGLE_MAX = 180.0


def make_crc_table(poly: int = 0x4D) -> tuple[int, ...]:
    """Create the CRC-8 lookup table used by the LD19."""

    table = []

    for value in range(256):
        crc = value

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

        table.append(crc)

    return tuple(table)


CRC_TABLE = make_crc_table()


def crc8(data: bytes) -> int:
    """Calculate an LD19 packet CRC."""

    crc = 0

    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]

    return crc


@dataclass(frozen=True)
class Point:
    """One LiDAR measurement."""

    angle_deg: float
    distance_m: float
    intensity: int


class LD19:
    """Read and decode LD19/D300 serial packets."""

    def __init__(
        self,
        port: str,
        baudrate: int = 230400,
        timeout: float = 0.1,
    ):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
        )

        self.buffer = bytearray()
        self.good_packets = 0
        self.bad_packets = 0

    def close(self) -> None:
        """Close the serial connection."""

        if self.serial.is_open:
            self.serial.close()

    def read_points(self) -> list[Point]:
        """Read available serial bytes and return decoded points."""

        number_of_bytes = self.serial.in_waiting or PACKET_SIZE
        self.buffer.extend(self.serial.read(number_of_bytes))

        output: list[Point] = []

        while len(self.buffer) >= PACKET_SIZE:

            # Look for the beginning of an LD19 packet.
            try:
                start = self.buffer.index(HEADER)
            except ValueError:
                self.buffer.clear()
                break

            # Remove bytes before the packet header.
            if start:
                del self.buffer[:start]

            if len(self.buffer) < PACKET_SIZE:
                break

            # The second packet byte should be 0x2C.
            if self.buffer[1] != VER_LEN:
                del self.buffer[0]
                continue

            packet = bytes(self.buffer[:PACKET_SIZE])

            # Reject damaged packets.
            if crc8(packet[:-1]) != packet[-1]:
                self.bad_packets += 1
                del self.buffer[0]
                continue

            del self.buffer[:PACKET_SIZE]

            self.good_packets += 1
            output.extend(self._decode(packet))

        return output

    @staticmethod
    def _decode(packet: bytes) -> list[Point]:
        """Decode the 12 measurements contained in one packet."""

        start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
        end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0

        # Handle the transition from 359 degrees back to 0 degrees.
        angle_span = (end_angle - start_angle) % 360.0

        points = []

        for index in range(SAMPLES_PER_PACKET):
            offset = 6 + index * 3

            distance_mm = struct.unpack_from("<H", packet, offset)[0]
            intensity = packet[offset + 2]

            angle = (
                start_angle
                + angle_span * index / (SAMPLES_PER_PACKET - 1)
            ) % 360.0

            # A distance of zero is an invalid measurement.
            if distance_mm > 0:
                points.append(
                    Point(
                        angle_deg=angle,
                        distance_m=distance_mm / 1000.0,
                        intensity=intensity,
                    )
                )

        return points


def point_xy(point: Point) -> tuple[float, float]:
    """Convert a polar LiDAR point to Cartesian coordinates."""

    angle_radians = math.radians(point.angle_deg)

    x = point.distance_m * math.cos(angle_radians)
    y = point.distance_m * math.sin(angle_radians)

    return x, y


def cluster_points(
    points: list[Point],
    gap_m: float,
    min_points: int,
) -> list[list[Point]]:
    """Group neighboring LiDAR returns into objects."""

    if not points:
        return []

    ordered_points = sorted(
        points,
        key=lambda point: point.angle_deg,
    )

    groups: list[list[Point]] = [[ordered_points[0]]]

    for point in ordered_points[1:]:
        previous_point = groups[-1][-1]

        previous_x, previous_y = point_xy(previous_point)
        current_x, current_y = point_xy(point)

        separation = math.hypot(
            current_x - previous_x,
            current_y - previous_y,
        )

        if separation <= gap_m:
            groups[-1].append(point)
        else:
            groups.append([point])

    return [
        group
        for group in groups
        if len(group) >= min_points
    ]


def cluster_width(cluster: list[Point]) -> float:
    """Estimate the visible width of a detected object."""

    coordinates = np.array(
        [point_xy(point) for point in cluster]
    )

    minimum = coordinates.min(axis=0)
    maximum = coordinates.max(axis=0)

    return float(np.linalg.norm(maximum - minimum))


def parse_args() -> argparse.Namespace:
    """Read command-line options."""

    parser = argparse.ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="LD19 serial device",
    )

    parser.add_argument(
        "--min-range",
        type=float,
        default=ONE_FOOT_METERS,
        help=(
            "Ignore objects closer than this many metres "
            "(default: 0.3048 metres or 1 foot)"
        ),
    )

    parser.add_argument(
        "--max-range",
        type=float,
        default=8.0,
        help="Maximum displayed distance in metres",
    )

    parser.add_argument(
        "--min-intensity",
        type=int,
        default=5,
        help="Minimum accepted LiDAR signal intensity",
    )

    parser.add_argument(
        "--cluster-gap",
        type=float,
        default=0.18,
        help="Maximum gap between points in the same object",
    )

    parser.add_argument(
        "--min-cluster-points",
        type=int,
        default=3,
        help="Minimum number of points required for an object",
    )

    parser.add_argument(
        "--human-min-width",
        type=float,
        default=0.08,
        help="Minimum human-candidate width in metres",
    )

    parser.add_argument(
        "--human-max-width",
        type=float,
        default=0.80,
        help="Maximum human-candidate width in metres",
    )

    parser.add_argument(
        "--flip",
        action="store_true",
        help="Mirror the radar direction",
    )

    return parser.parse_args()


def main() -> None:
    """Run the live radar."""

    args = parse_args()

    if args.min_range < 0:
        raise ValueError("--min-range cannot be negative")

    if args.min_range >= args.max_range:
        raise ValueError(
            "--min-range must be smaller than --max-range"
        )

    lidar = LD19(args.port)

    # Store one measurement every 0.5 degrees.
    bins: list[Point | None] = [None] * 720

    last_data_time = time.monotonic()

    figure, axis = plt.subplots(
        subplot_kw={"projection": "polar"},
        figsize=(9, 6),
    )

    # Place zero degrees directly in front of the LiDAR.
    axis.set_theta_zero_location("N")

    if args.flip:
        axis.set_theta_direction(1)
    else:
        axis.set_theta_direction(-1)

    # Only display 0 through 180 degrees.
    axis.set_thetamin(DETECTION_ANGLE_MIN)
    axis.set_thetamax(DETECTION_ANGLE_MAX)

    axis.set_ylim(0, args.max_range)

    axis.set_title(
        "LD19 / D300 Front 180-Degree Radar\n"
        "Minimum detection distance: 1 foot"
    )

    object_plot = axis.scatter(
        [],
        [],
        s=10,
        color="#42a5f5",
        label="Object",
    )

    human_plot = axis.scatter(
        [],
        [],
        s=28,
        color="#ff5252",
        label="Human candidate",
    )

    status_text = axis.text(
        0.02,
        0.02,
        "Waiting for LiDAR data...",
        transform=axis.transAxes,
    )

    axis.legend(loc="upper right")

    def update(_frame):
        nonlocal last_data_time

        incoming_points = lidar.read_points()

        if incoming_points:
            last_data_time = time.monotonic()

        for point in incoming_points:

            # Allow only the front 180-degree region.
            angle_allowed = (
                DETECTION_ANGLE_MIN
                <= point.angle_deg
                <= DETECTION_ANGLE_MAX
            )

            # Ignore anything closer than one foot.
            distance_allowed = (
                args.min_range
                <= point.distance_m
                <= args.max_range
            )

            intensity_allowed = (
                point.intensity >= args.min_intensity
            )

            if (
                angle_allowed
                and distance_allowed
                and intensity_allowed
            ):
                bin_number = round(point.angle_deg * 2)
                bin_number %= len(bins)

                bins[bin_number] = point

        visible_points = [
            point
            for point in bins
            if point is not None
        ]

        clusters = cluster_points(
            visible_points,
            gap_m=args.cluster_gap,
            min_points=args.min_cluster_points,
        )

        human_point_ids: set[int] = set()
        human_count = 0

        for cluster in clusters:
            width = cluster_width(cluster)

            if (
                args.human_min_width
                <= width
                <= args.human_max_width
            ):
                human_count += 1

                for point in cluster:
                    human_point_ids.add(id(point))

        object_points = [
            point
            for point in visible_points
            if id(point) not in human_point_ids
        ]

        human_points = [
            point
            for point in visible_points
            if id(point) in human_point_ids
        ]

        def make_offsets(
            selected_points: list[Point],
        ) -> np.ndarray:

            if not selected_points:
                return np.empty((0, 2))

            return np.array(
                [
                    (
                        math.radians(point.angle_deg),
                        point.distance_m,
                    )
                    for point in selected_points
                ]
            )

        object_plot.set_offsets(
            make_offsets(object_points)
        )

        human_plot.set_offsets(
            make_offsets(human_points)
        )

        data_age = time.monotonic() - last_data_time

        status_text.set_text(
            f"Points: {len(visible_points)}\n"
            f"Human candidates: {human_count}\n"
            f"Valid packets: {lidar.good_packets}\n"
            f"CRC errors: {lidar.bad_packets}\n"
            f"Data age: {data_age:.1f} seconds"
        )

        return object_plot, human_plot, status_text

    animation = FuncAnimation(
        figure,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()

    finally:
        # Retain animation until the window closes.
        _ = animation
        lidar.close()


if __name__ == "__main__":
    main()