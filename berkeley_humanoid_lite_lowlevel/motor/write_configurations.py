# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import json
import time

import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
transport = recoil.SocketCANTransport(port=args.port, baudrate=1000000)
transport.start()

motor = recoil.MotorController(transport, id=args.id)


motor_configuration = json.load(open("motor_configuration.json"))

delay_t = 0.1


print(f"Writing configuration to actuator #{args.id}")

config = {
    "position_controller": {},
    "current_controller": {},
    "powerstage": {},
    "motor": {},
    "encoder": {},
}

motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO, motor_configuration["position_controller"]["gear_ratio"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_KP, motor_configuration["position_controller"]["position_kp"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_KI, motor_configuration["position_controller"]["position_ki"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP, motor_configuration["position_controller"]["velocity_kp"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KI, motor_configuration["position_controller"]["velocity_ki"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT, motor_configuration["position_controller"]["torque_limit"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT, motor_configuration["position_controller"]["velocity_limit"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER, motor_configuration["position_controller"]["position_limit_upper"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER, motor_configuration["position_controller"]["position_limit_lower"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_POSITION_OFFSET, motor_configuration["position_controller"]["position_offset"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA, motor_configuration["position_controller"]["torque_filter_alpha"])
time.sleep(delay_t)

motor._write_parameter_f32(recoil.Parameter.CURRENT_CONTROLLER_I_LIMIT, motor_configuration["current_controller"]["i_limit"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.CURRENT_CONTROLLER_I_KP, motor_configuration["current_controller"]["i_kp"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.CURRENT_CONTROLLER_I_KI, motor_configuration["current_controller"]["i_ki"])
time.sleep(delay_t)

motor._write_parameter_f32(recoil.Parameter.POWERSTAGE_UNDERVOLTAGE_THRESHOLD, motor_configuration["powerstage"]["undervoltage_threshold"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POWERSTAGE_OVERVOLTAGE_THRESHOLD, motor_configuration["powerstage"]["overvoltage_threshold"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA, motor_configuration["powerstage"]["bus_voltage_filter_alpha"])
time.sleep(delay_t)

motor._write_parameter_u32(recoil.Parameter.MOTOR_POLE_PAIRS, motor_configuration["motor"]["pole_pairs"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.MOTOR_TORQUE_CONSTANT, motor_configuration["motor"]["torque_constant"])
time.sleep(delay_t)
motor._write_parameter_i32(recoil.Parameter.MOTOR_PHASE_ORDER, motor_configuration["motor"]["phase_order"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.MOTOR_MAX_CALIBRATION_CURRENT, motor_configuration["motor"]["max_calibration_current"])
time.sleep(delay_t)

motor._write_parameter_u32(recoil.Parameter.ENCODER_CPR, motor_configuration["encoder"]["cpr"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.ENCODER_POSITION_OFFSET, motor_configuration["encoder"]["position_offset"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.ENCODER_VELOCITY_FILTER_ALPHA, motor_configuration["encoder"]["velocity_filter_alpha"])
time.sleep(delay_t)
motor._write_parameter_f32(recoil.Parameter.ENCODER_FLUX_OFFSET, motor_configuration["encoder"]["flux_offset"])
time.sleep(delay_t)

print(" storing to flash")
motor.store_settings_to_flash()
time.sleep(0.2)

transport.stop()

print("Done")
