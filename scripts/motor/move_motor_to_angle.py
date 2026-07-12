# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse
import time

from loop_rate_limiters import RateLimiter


# Same sign convention as scripts/calibrate_joints.py for leg joints.
JOINT_AXIS_DIRECTIONS = {
    ("can2", 1): -1,
    ("can2", 3): +1,
    ("can2", 5): -1,
    ("can2", 7): -1,
    ("can2", 11): -1,
    ("can2", 13): +1,
    ("can3", 2): -1,
    ("can3", 4): +1,
    ("can3", 6): +1,
    ("can3", 8): +1,
    ("can3", 12): +1,
    ("can3", 14): +1,
}


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--channel", required=True, help="CAN channel, e.g. can2")
    parser.add_argument("-i", "--id", required=True, type=int, help="motor device ID")
    parser.add_argument("-a", "--angle", required=True, type=float, help="target joint angle in radians")
    parser.add_argument("--kp", type=float, default=0.2)
    parser.add_argument("--kd", type=float, default=0.005)
    parser.add_argument("--torque-limit", type=float, default=0.2)
    parser.add_argument("--velocity", type=float, default=0.0, help="target joint velocity in rad/s")
    parser.add_argument("--rate", type=float, default=200.0, help="command rate in Hz")
    parser.add_argument("--duration", type=float, help="seconds to hold target; omit to hold until Ctrl+C")
    return parser.parse_args()


def release_motor(bus, recoil, device_id):
    try:
        bus.set_mode(device_id, recoil.Mode.IDLE)
    except Exception as exc:
        print(f"warning: failed to release motor {device_id}: {exc}")
    finally:
        bus.stop()


def main():
    args = get_args()

    import berkeley_humanoid_lite_lowlevel.recoil as recoil

    direction = JOINT_AXIS_DIRECTIONS.get((args.channel, args.id), 1)
    target = args.angle * direction
    velocity_target = args.velocity * direction
    rate = RateLimiter(frequency=args.rate)

    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    try:
        bus.write_position_kp(args.id, args.kp)
        bus.write_position_kd(args.id, args.kd)
        bus.write_torque_limit(args.id, args.torque_limit)
        bus.set_mode(args.id, recoil.Mode.POSITION)
        bus.feed(args.id)

        print(f"Moving {args.channel}:{args.id} to {args.angle:.4f} rad. Press Ctrl+C to release.")
        deadline = None if args.duration is None else time.monotonic() + args.duration

        while deadline is None or time.monotonic() < deadline:
            position, velocity = bus.write_read_pdo_2(args.id, target, velocity_target)
            if position is not None and velocity is not None:
                print(f"position: {position * direction: .4f} rad  velocity: {velocity * direction: .4f} rad/s")
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        release_motor(bus, recoil, args.id)


if __name__ == "__main__":
    main()
