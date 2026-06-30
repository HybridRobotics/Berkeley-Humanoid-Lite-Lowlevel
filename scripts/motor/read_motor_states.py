# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse
import time


# ponytail: keep the static map here to avoid Humanoid(), which starts IMU/gamepad.
HUMANOID_JOINTS = [
    ("can0", 1, "left_shoulder_pitch_joint"),
    ("can0", 3, "left_shoulder_roll_joint"),
    ("can0", 5, "left_shoulder_yaw_joint"),
    ("can0", 7, "left_elbow_pitch_joint"),
    ("can0", 9, "left_wrist_yaw_joint"),
    ("can1", 2, "right_shoulder_pitch_joint"),
    ("can1", 4, "right_shoulder_roll_joint"),
    ("can1", 6, "right_shoulder_yaw_joint"),
    ("can1", 8, "right_elbow_pitch_joint"),
    ("can1", 10, "right_wrist_yaw_joint"),
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


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--channel", default="can0", help="CAN channel to use with --ids")
    parser.add_argument("-i", "--ids", nargs="+", type=int, help="read these motor IDs on --channel")
    parser.add_argument("--interval", type=float, default=0.1, help="delay between read loops in seconds")
    parser.add_argument("--seed-timeout", type=float, default=0.1, help="initial SDO read timeout in seconds")
    return parser.parse_args()


def seed_target_position(recoil, bus, device_id, timeout):
    return bus._read_parameter_f32(
        device_id,
        recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED,
        timeout=timeout,
    )


def read_state(bus, device_id, target_position):
    return bus.write_read_pdo_2(device_id, target_position, 0.0)


def main():
    args = get_args()
    joints = [(args.channel, device_id, f"motor_{device_id}") for device_id in args.ids] if args.ids else HUMANOID_JOINTS

    import berkeley_humanoid_lite_lowlevel.recoil as recoil

    buses = {}
    target_positions = {}
    try:
        while True:
            for channel, device_id, name in joints:
                if channel not in buses:
                    buses[channel] = recoil.Bus(channel=channel, bitrate=1000000)

                key = (channel, device_id)
                if key not in target_positions:
                    target_positions[key] = seed_target_position(recoil, buses[channel], device_id, args.seed_timeout)
                if target_positions[key] is None:
                    position, velocity = None, None
                else:
                    position, velocity = read_state(buses[channel], device_id, target_positions[key])

                if position is None or velocity is None:
                    print(f"{channel}:{device_id:<2} {name:<32} position: ---      velocity: ---")
                else:
                    target_positions[key] = position
                    print(f"{channel}:{device_id:<2} {name:<32} position: {position: .6f}  velocity: {velocity: .6f}")

            print()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        for bus in buses.values():
            bus.stop()


if __name__ == "__main__":
    main()
