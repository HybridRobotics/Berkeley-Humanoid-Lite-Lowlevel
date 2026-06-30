"""
calibrate_joints.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Run this script after each power cycle to calibrate the encoder offset of each joint.
"""

import argparse
from pathlib import Path
import select
import sys
import time

import numpy as np
import yaml


JOINTS = [
    ("can2", 1, "left_hip_roll_joint"),
    ("can2", 3, "left_hip_yaw_joint"),
    ("can2", 5, "left_hip_pitch_joint"),
    ("can2", 7, "left_knee_pitch_joint"),
    ("can2", 11, "left_ankle_pitch_joint"),
    ("can2", 13, "left_ankle_roll_joint"),
    ("can3", 2, "right_hip_roll_joint"),
    ("can3", 4, "right_hip_yaw_joint"),
    ("can3", 6, "right_hip_pitch_joint"),
    ("can3", 8, "right_knee_pitch_joint"),
    ("can3", 12, "right_ankle_pitch_joint"),
    ("can3", 14, "right_ankle_roll_joint"),
]

JOINT_AXIS_DIRECTIONS = np.array([
    -1, +1, -1,
    -1,
    -1, +1,
    -1, +1, +1,
    +1,
    +1, +1
])

IDEAL_VALUES = np.array([
    np.deg2rad(-(10)),
    np.deg2rad(+(33.75)),
    np.deg2rad(+(56.25)),
    np.deg2rad(+(0)),
    np.deg2rad(-(45)),
    np.deg2rad(-(15)),

    np.deg2rad(+(10)),
    np.deg2rad(-(33.75)),
    np.deg2rad(+(56.25)),
    np.deg2rad(+(0)),
    np.deg2rad(-(45)),
    np.deg2rad(+(15)),
])

LIMIT_DIRECTIONS = [
    "min", "max", "min",  # can2 id5 originally "max"
    "min",
    "min", "min",
    "max", "min", "min",  # can3 id6 originally "max"
    "min",
    "min", "max",
]

CALIBRATION_PATH = Path("calibration.yaml")


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--channel", choices=[f"can{i}" for i in range(4)], help="only calibrate a motor on this CAN channel")
    parser.add_argument("-i", "--id", type=int, help="only calibrate this motor device ID")
    parser.add_argument("--timeout", type=float, default=0.02, help="CAN read timeout per joint in seconds")
    parser.add_argument("--interval", type=float, default=0.05, help="delay between read loops in seconds")
    args = parser.parse_args()
    if (args.channel is None) != (args.id is None):
        parser.error("-c/--channel and -i/--id must be used together")
    if args.channel is not None and not selected_joint_indices(args.channel, args.id):
        parser.error(f"no calibratable joint for {args.channel} device_id={args.id}")
    return args


def selected_joint_indices(channel, device_id):
    if channel is None and device_id is None:
        return list(range(len(JOINTS)))
    return [i for i, (joint_channel, joint_id, _) in enumerate(JOINTS) if joint_channel == channel and joint_id == device_id]


def read_joint_position(recoil, buses, joint_index, timeout):
    channel, device_id, _ = JOINTS[joint_index]
    reading = buses[channel]._read_parameter_f32(
        device_id,
        recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED,
        timeout=timeout,
    )
    return np.nan if reading is None else reading * JOINT_AXIS_DIRECTIONS[joint_index]


def update_limit_reading(limit_reading, joint_reading, limit_direction):
    if np.isnan(joint_reading):
        return limit_reading
    if np.isnan(limit_reading):
        return joint_reading
    return min(limit_reading, joint_reading) if limit_direction == "min" else max(limit_reading, joint_reading)


def format_reading(reading):
    return "---" if np.isnan(reading) else f"{reading:.2f}"


def stop_buses(recoil, buses):
    for channel, device_id, _ in JOINTS:
        if channel in buses:
            buses[channel].set_mode(device_id, recoil.Mode.IDLE)
    for bus in buses.values():
        bus.stop()


def load_position_offsets(path):
    if not path.exists():
        return None
    offsets = np.array((yaml.safe_load(path.read_text()) or {}).get("position_offsets", []), dtype=float)
    return offsets if offsets.shape == (len(JOINTS),) else None


def main():
    args = get_args()
    joint_indices = selected_joint_indices(args.channel, args.id)

    import berkeley_humanoid_lite_lowlevel.recoil as recoil

    buses = {channel: recoil.Bus(channel=channel, bitrate=1000000) for channel in sorted({JOINTS[i][0] for i in joint_indices})}
    try:
        limit_readings = np.full(len(JOINTS), np.nan)

        try:
            for count, i in enumerate(joint_indices, 1):
                channel, device_id, joint_name = JOINTS[i]
                limit_direction = LIMIT_DIRECTIONS[i]
                print()
                print(f"[{count}/{len(joint_indices)}] {channel} device_id={device_id} {joint_name}", flush=True)
                print(f"Move this joint to joint {limit_direction.upper()} limit. Press Enter to accept.", flush=True)

                while not select.select([sys.stdin], [], [], 0)[0]:
                    print(f"Move this joint to joint {limit_direction.upper()} limit. Press Enter to accept.", flush=True)
                    joint_reading = read_joint_position(recoil, buses, i, args.timeout)
                    limit_readings[i] = update_limit_reading(limit_readings[i], joint_reading, limit_direction)
                    print(
                        f"{time.time():.3f} current={format_reading(joint_reading)} limit={format_reading(limit_readings[i])}",
                        flush=True,
                    )
                    time.sleep(args.interval)

                sys.stdin.readline()
        except KeyboardInterrupt:
            pass

        if np.isnan(limit_readings[joint_indices]).any():
            missing = [JOINTS[i][2] for i in joint_indices if np.isnan(limit_readings[i])]
            print(f"ERROR: no readings for: {', '.join(missing)}", file=sys.stderr)
            return 1

        print("final readings at the limits:")
        print([f"{limit_readings[i]:.4f}" for i in joint_indices])

        offsets = np.full(len(JOINTS), np.nan)
        offsets[joint_indices] = limit_readings[joint_indices] - IDEAL_VALUES[joint_indices]
        print("offsets:")
        print([f"{offsets[i]:.4f}" for i in joint_indices])

        if len(joint_indices) != len(JOINTS):
            existing_offsets = load_position_offsets(CALIBRATION_PATH)
            if existing_offsets is None:
                print(f"ERROR: {CALIBRATION_PATH} with {len(JOINTS)} position_offsets is required for single-motor calibration", file=sys.stderr)
                return 1
            existing_offsets[joint_indices] = offsets[joint_indices]
            offsets = existing_offsets

        with CALIBRATION_PATH.open("w") as f:
            yaml.dump({"position_offsets": [float(offset) for offset in offsets]}, f)
    finally:
        stop_buses(recoil, buses)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
