# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import json
import time

import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
transport = recoil.SocketCANTransport(port=args.port, baudrate=1000000)
transport.start()

motor = recoil.MotorController(transport, id=args.id)



print(f"Reading configuration from actuator #{args.id}")

config = {
    "position_controller": {},
    "current_controller": {},
    "powerstage": {},
    "motor": {},
    "encoder": {},
}

config["device_id"] = motor._read_parameter_u32(recoil.Parameter.DEVICE_ID)
config["firmware_version"] = hex(motor._read_parameter_u32(recoil.Parameter.FIRMWARE_VERSION))
config["watchdog_timeout"] = motor._read_parameter_u32(recoil.Parameter.WATCHDOG_TIMEOUT)
config["fast_frame_frequency"] = motor.read_fast_frame_frequency()

config["position_controller"]["gear_ratio"] = motor._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO)
config["position_controller"]["position_kp"] = motor.read_position_kp()
config["position_controller"]["position_ki"] = motor.read_position_ki()
config["position_controller"]["velocity_kp"] = motor.read_velocity_kp()
config["position_controller"]["velocity_ki"] = motor.read_velocity_ki()
config["position_controller"]["torque_limit"] = motor.read_torque_limit()
config["position_controller"]["velocity_limit"] = motor.read_velocity_limit()
config["position_controller"]["position_limit_upper"] = motor.read_position_limit_upper()
config["position_controller"]["position_limit_lower"] = motor.read_position_limit_lower()
config["position_controller"]["position_offset"] = motor.read_position_offset()
config["position_controller"]["torque_filter_alpha"] = motor.read_torque_filter_alpha()

config["current_controller"]["i_limit"] = motor.read_current_limit()
config["current_controller"]["i_kp"] = motor.read_current_kp()
config["current_controller"]["i_ki"] = motor.read_current_ki()

config["powerstage"]["undervoltage_threshold"] = motor._read_parameter_f32(recoil.Parameter.POWERSTAGE_UNDERVOLTAGE_THRESHOLD)
config["powerstage"]["overvoltage_threshold"] = motor._read_parameter_f32(recoil.Parameter.POWERSTAGE_OVERVOLTAGE_THRESHOLD)
config["powerstage"]["bus_voltage_filter_alpha"] = motor.read_bus_voltage_filter_alpha()

config["motor"]["pole_pairs"] = motor.read_motor_pole_pairs()
config["motor"]["torque_constant"] = motor.read_motor_torque_constant()
config["motor"]["phase_order"] = motor.read_motor_phase_order()
config["motor"]["max_calibration_current"] = motor.read_motor_calibration_current()

config["encoder"]["cpr"] = motor.read_encoder_cpr()
config["encoder"]["position_offset"] = motor.read_encoder_position_offset()
config["encoder"]["velocity_filter_alpha"] = motor.read_encoder_velocity_filter_alpha()
config["encoder"]["flux_offset"] = motor.read_encoder_flux_offset()


time.sleep(0.1)


with open("motor_configuration.json", "w") as f:
    json.dump(config, f, indent=4)

transport.stop()

print("Done")
