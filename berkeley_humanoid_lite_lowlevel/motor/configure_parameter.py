# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
transport = recoil.SocketCANTransport(port=args.port, baudrate=1000000)
transport.start()

motor = recoil.MotorController(transport, id=args.id)


def configure_fast_frame_rate(motor: recoil.MotorController, fast_frame_rate: int):
    print("Rate (before):\t", motor._read_parameter_u32(recoil.Parameter.FAST_FRAME_FREQUENCY))
    motor._write_parameter_u32(recoil.Parameter.FAST_FRAME_FREQUENCY, fast_frame_rate)
    print("Rate (updated):\t", motor._read_parameter_u32(recoil.Parameter.FAST_FRAME_FREQUENCY))


def configure_gear_ratio(motor: recoil.MotorController, gear_ratio: float):
    print("Gear Ratio (before):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO))
    motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO, -gear_ratio)
    print("Gear Ratio (updated):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO))


def configure_position_pd(motor: recoil.MotorController, kp: float, kd: float):
    print("Kp (before):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_KP))
    motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_KP, kp)
    print("Kp (updated):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_KP))

    print("Kd (before):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP))
    motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP, kd)
    print("Kd (updated):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP))


def configure_torque_limit(motor: recoil.MotorController, torque_limit: float):
    print("Torque Limit (before):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT))
    motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT, torque_limit)
    print("Torque Limit (updated):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT))


def configure_phase_order(motor: recoil.MotorController, phase_order: int):
    print("Phase order (before):\t", motor._read_parameter_i32(recoil.Parameter.MOTOR_PHASE_ORDER))
    motor._write_parameter_i32(recoil.Parameter.MOTOR_PHASE_ORDER, phase_order)
    print("Phase order (updated):\t", motor._read_parameter_i32(recoil.Parameter.MOTOR_PHASE_ORDER))



def configure_position_limit(motor: recoil.MotorController, lower_limit: float, upper_limit: float):
    print("Position Lower Limit (before):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER))
    motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER, lower_limit)
    print("Position Lower Limit (updated):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER))

    print("Position Upper Limit (before):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER))
    motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER, upper_limit)
    print("Position Upper Limit (updated):\t", motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER))


def configure_current_bandwidth(motor: recoil.MotorController, bandwidth: float, kp: float = 0.0001, ki: float = 0.000001):
    print("Previous current kp:\t", motor._read_parameter_f32(recoil.Parameter.CURRENT_CONTROLLER_I_KP))
    print("Previous current ki:\t", motor._read_parameter_f32(recoil.Parameter.CURRENT_CONTROLLER_I_KI))

    motor.set_current_bandwidth(bandwidth, kp, ki)

    print("New current kp:\t", motor._read_parameter_f32(recoil.Parameter.CURRENT_CONTROLLER_I_KP))
    print("New current ki:\t", motor._read_parameter_f32(recoil.Parameter.CURRENT_CONTROLLER_I_KI))

def store_to_flash(motor: recoil.MotorController):
    motor.store_setting_to_flash()
    print("Settings stored to flash!")


# configure_fast_frame_rate(motor, 0)

# configure_gear_ratio(motor, 15)

# configure_position_pd(motor, 20, 1)

# configure_torque_limit(motor, 2)

# configure_phase_order(motor, -1)

# pi = math.pi

# d90 = 0.5 * pi
# d60 = 1./3. * math.pi
# d45 = 0.25 * math.pi
# d30 = 1./6. * math.pi
# d10 = 0.5 * pi/9

# configure_position_limit(motor, -d90, d90)

# configure_current_bandwidth(motor, 100)


# store_to_flash(motor)


transport.stop()
