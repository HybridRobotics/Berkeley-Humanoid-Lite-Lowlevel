# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import json
import time

import berkeley_humanoid_lite_lowlevel.recoil as recoil

from robot import ROBOT


robot_configuration = {}


ROBOT.check_connection()

for entry in ROBOT.joints:
    joint_name, joint = entry

    print(f"Reading configuration for {joint_name}")
    
    config = {
        "position_controller": {},
        "current_controller": {},
        "powerstage": {},
        "motor": {},
        "encoder": {},
    }

    config["device_id"] = joint._read_parameter_u32(recoil.Parameter.DEVICE_ID)
    config["firmware_version"] = hex(joint._read_parameter_u32(recoil.Parameter.FIRMWARE_VERSION))
    config["watchdog_timeout"] = joint._read_parameter_u32(recoil.Parameter.WATCHDOG_TIMEOUT)
    config["fast_frame_frequency"] = joint.read_fast_frame_frequency()
    
    config["position_controller"]["gear_ratio"] = joint._read_parameter_f32(recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO)
    config["position_controller"]["position_kp"] = joint.read_position_kp()
    config["position_controller"]["position_ki"] = joint.read_position_ki()
    config["position_controller"]["velocity_kp"] = joint.read_velocity_kp()
    config["position_controller"]["velocity_ki"] = joint.read_velocity_ki()
    config["position_controller"]["torque_limit"] = joint.read_torque_limit()
    config["position_controller"]["velocity_limit"] = joint.read_velocity_limit()
    config["position_controller"]["position_limit_upper"] = joint.read_position_limit_upper()
    config["position_controller"]["position_limit_lower"] = joint.read_position_limit_lower()
    config["position_controller"]["position_offset"] = joint.read_position_offset()
    config["position_controller"]["torque_filter_alpha"] = joint.read_torque_filter_alpha()

    config["current_controller"]["i_limit"] = joint.read_current_limit()
    config["current_controller"]["i_kp"] = joint.read_current_kp()
    config["current_controller"]["i_ki"] = joint.read_current_ki()
    
    config["powerstage"]["undervoltage_threshold"] = joint._read_parameter_f32(recoil.Parameter.POWERSTAGE_UNDERVOLTAGE_THRESHOLD)
    config["powerstage"]["overvoltage_threshold"] = joint._read_parameter_f32(recoil.Parameter.POWERSTAGE_OVERVOLTAGE_THRESHOLD)
    config["powerstage"]["bus_voltage_filter_alpha"] = joint.read_bus_voltage_filter_alpha()
    
    config["motor"]["pole_pairs"] = joint.read_motor_pole_pairs()
    config["motor"]["torque_constant"] = joint.read_motor_torque_constant()
    config["motor"]["phase_order"] = joint.read_motor_phase_order()
    config["motor"]["max_calibration_current"] = joint.read_motor_calibration_current()
    
    config["encoder"]["cpr"] = joint.read_encoder_cpr()
    config["encoder"]["position_offset"] = joint.read_encoder_position_offset()
    config["encoder"]["velocity_filter_alpha"] = joint.read_encoder_velocity_filter_alpha()
    config["encoder"]["flux_offset"] = joint.read_encoder_flux_offset()
    
    robot_configuration[joint_name] = config
    time.sleep(0.1)


with open("robot_configuration.json", "w") as f:
    json.dump(robot_configuration, f, indent=4)

ROBOT.stop()

print("Done")
