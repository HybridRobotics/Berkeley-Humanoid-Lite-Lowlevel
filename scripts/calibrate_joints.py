"""
calibrate_joints.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Run this script after each power cycle to calibrate the encoder offset of each joint.
"""

import argparse
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
    "min", "max", "max",
    "min",
    "min", "min",
    "max", "min", "max",
    "min",
    "min", "max",
]


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=0.02, help="CAN read timeout per joint in seconds")
    parser.add_argument("--interval", type=float, default=0.05, help="delay between read loops in seconds")
    return parser.parse_args()


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
        buses[channel].set_mode(device_id, recoil.Mode.IDLE)
    for bus in buses.values():
        bus.stop()


def main():
    args = get_args()

    import berkeley_humanoid_lite_lowlevel.recoil as recoil

    buses = {channel: recoil.Bus(channel=channel, bitrate=1000000) for channel in sorted({joint[0] for joint in JOINTS})}
    try:
        limit_readings = np.full(len(JOINTS), np.nan)

        try:
            for i, (channel, device_id, joint_name) in enumerate(JOINTS):
                limit_direction = LIMIT_DIRECTIONS[i]
                print()
                print(f"[{i + 1}/{len(JOINTS)}] {channel} device_id={device_id} {joint_name}", flush=True)
                print(f"Move this joint to joint {limit_direction.upper()} limit. Press Enter to accept.", flush=True)

                while not select.select([sys.stdin], [], [], 0)[0]:
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

        if np.isnan(limit_readings).any():
            missing = [name for (_, _, name), reading in zip(JOINTS, limit_readings) if np.isnan(reading)]
            print(f"ERROR: no readings for: {', '.join(missing)}", file=sys.stderr)
            return 1

        print("final readings at the limits:")
        print([f"{limit:.4f}" for limit in limit_readings])

        offsets = limit_readings - IDEAL_VALUES
        print("offsets:")
        print([f"{offset:.4f}" for offset in offsets])

        with open("calibration.yaml", "w") as f:
            yaml.dump({"position_offsets": [float(offset) for offset in offsets]}, f)
    finally:
        stop_buses(recoil, buses)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
