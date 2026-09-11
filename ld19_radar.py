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
SIX_FEET_METERS = 1.8288
DETECTION_RIGHT_MAX = 90.0
DETECTION_LEFT_MIN = 270.0

# Motion tracking settings
POINT_MAX_AGE_SECONDS = 0.30
MOTION_SAMPLE_SECONDS = 0.15
MIN_MOVEMENT_METERS = 0.025
MAX_TRACK_MATCH_METERS = 0.45
TRACK_LOST_TIMEOUT_SECONDS = 1.00
CENTER_DEAD_ZONE_METERS = 0.12


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

    # Join a physical object that crosses the 359-to-0-degree boundary.
    if len(groups) > 1:
        last_x, last_y = point_xy(groups[-1][-1])
        first_x, first_y = point_xy(groups[0][0])
        wraparound_gap = math.hypot(
            first_x - last_x,
            first_y - last_y,
        )

        if wraparound_gap <= gap_m:
            groups[0] = groups[-1] + groups[0]
            groups.pop()

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


def cluster_center(cluster: list[Point]) -> tuple[float, float]:
    """Return the average Cartesian center of a cluster."""

    coordinates = np.array(
        [point_xy(point) for point in cluster]
    )

    center = coordinates.mean(axis=0)
    return float(center[0]), float(center[1])


def center_distance(
    first: tuple[float, float],
    second: tuple[float, float],
) -> float:
    """Return the distance between two cluster centers."""

    return math.hypot(
        first[0] - second[0],
        first[1] - second[1],
    )


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
        "--max-range",
        type=float,
        default=SIX_FEET_METERS,
        help="Maximum displayed distance in metres (default: 1.8288 m / 6 ft)",
    )

    parser.add_argument(
        "--center-dead-zone",
        type=float,
        default=CENTER_DEAD_ZONE_METERS,
        help="Hide near-origin returns inside this radius in metres (default: 0.12)",
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

    if args.max_range <= 0:
        raise ValueError("--max-range must be greater than zero")

    if args.center_dead_zone < 0:
        raise ValueError("--center-dead-zone cannot be negative")

    if args.center_dead_zone >= args.max_range:
        raise ValueError("--center-dead-zone must be smaller than --max-range")

    lidar = LD19(args.port)

    # Store one measurement every 0.5 degrees.
    bins: list[tuple[Point, float] | None] = [None] * 720

    last_data_time = time.monotonic()
    previous_candidate_centers: list[tuple[float, float]] = []
    last_motion_sample_time = 0.0
    selected_center: tuple[float, float] | None = None
    selected_until = 0.0

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

    # Show the wraparound front sector: 270-360 degrees and 0-90 degrees.
    # Matplotlib renders 270 degrees as -90 degrees in this centered view.
    axis.set_thetamin(-90.0)
    axis.set_thetamax(90.0)

    axis.set_ylim(0, args.max_range)

    axis.set_title(
        "LD19 / D300 Front Radar (270° to 90°)\n"
        f"Zoomed range: 0-{args.max_range:.2f} metres"
    )

    object_plot = axis.scatter(
        [],
        [],
        s=3,
        color="#42a5f5",
        label="Object",
    )

    human_plot = axis.scatter(
        [],
        [],
        s=10,
        color="#ff5252",
        label="Closest moving human",
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
        nonlocal previous_candidate_centers
        nonlocal last_motion_sample_time
        nonlocal selected_center
        nonlocal selected_until

        current_time = time.monotonic()

        incoming_points = lidar.read_points()

        if incoming_points:
            last_data_time = current_time

        for point in incoming_points:

            # Allow 270-360 degrees OR 0-90 degrees.
            angle_allowed = (
                point.angle_deg >= DETECTION_LEFT_MIN
                or point.angle_deg <= DETECTION_RIGHT_MAX
            )

            # Remove only near-origin self-reflections/noise, not the old
            # one-foot exclusion zone.
            distance_allowed = (
                args.center_dead_zone
                < point.distance_m
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

                bins[bin_number] = (point, current_time)

        visible_points = [
            entry[0]
            for entry in bins
            if (
                entry is not None
                and current_time - entry[1] <= POINT_MAX_AGE_SECONDS
            )
        ]

        clusters = cluster_points(
            visible_points,
            gap_m=args.cluster_gap,
            min_points=args.min_cluster_points,
        )

        human_sized_clusters = [
            cluster
            for cluster in clusters
            if (
                args.human_min_width
                <= cluster_width(cluster)
                <= args.human_max_width
            )
        ]

        candidate_data = [
            (
                cluster,
                cluster_center(cluster),
                min(point.distance_m for point in cluster),
            )
            for cluster in human_sized_clusters
        ]

        # Compare cluster centers at a fixed interval. A candidate must move
        # enough to exceed LiDAR jitter while remaining close enough to be the
        # same physical cluster.
        if current_time - last_motion_sample_time >= MOTION_SAMPLE_SECONDS:
            moving_candidates = []

            for cluster, center, distance in candidate_data:
                if not previous_candidate_centers:
                    continue

                displacement = min(
                    center_distance(center, old_center)
                    for old_center in previous_candidate_centers
                )

                if (
                    MIN_MOVEMENT_METERS
                    <= displacement
                    <= MAX_TRACK_MATCH_METERS
                ):
                    moving_candidates.append(
                        (distance, center, cluster)
                    )

            # Select exactly one: the moving candidate closest to the LiDAR.
            if (
                moving_candidates
                and (
                    selected_center is None
                    or current_time > selected_until
                )
            ):
                _, selected_center, _ = min(
                    moving_candidates,
                    key=lambda candidate: candidate[0],
                )
                selected_until = (
                    current_time + TRACK_LOST_TIMEOUT_SECONDS
                )

            previous_candidate_centers = [
                center
                for _, center, _ in candidate_data
            ]
            last_motion_sample_time = current_time

        selected_cluster: list[Point] | None = None

        # Match the previously selected moving person to the current scan.
        if (
            selected_center is not None
            and current_time <= selected_until
            and candidate_data
        ):
            closest_match = min(
                candidate_data,
                key=lambda candidate: center_distance(
                    candidate[1],
                    selected_center,
                ),
            )

            match_distance = center_distance(
                closest_match[1],
                selected_center,
            )

            if match_distance <= MAX_TRACK_MATCH_METERS:
                selected_cluster = closest_match[0]
                selected_center = closest_match[1]

                # Refresh the lock whenever the same cluster is still visible.
                # It no longer needs to keep moving after initial acquisition.
                selected_until = (
                    current_time + TRACK_LOST_TIMEOUT_SECONDS
                )

        if (
            selected_cluster is None
            and current_time > selected_until
        ):
            selected_center = None

        human_point_ids = {
            id(point)
            for point in (selected_cluster or [])
        }

        human_count = 1 if selected_cluster else 0

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

        data_age = current_time - last_data_time

        status_text.set_text(
            f"Points: {len(visible_points)}\n"
            f"Closest moving human selected: {human_count}\n"
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
