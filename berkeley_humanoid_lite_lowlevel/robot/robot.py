# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time

from omegaconf import DictConfig, ListConfig, OmegaConf
import numpy as np

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.robot.imu import SerialImu, Baudrate
from berkeley_humanoid_lite_lowlevel.policy.gamepad import Se2Gamepad


class State:
    INVALID = 0
    IDLE = 1
    RL_INIT = 2
    RL_RUNNING = 3


def linear_interpolate(start: float, end: float, percentage: float) -> float:
    percentage = min(max(percentage, 0.0), 1.0)
    target = start * (1. - percentage) + end * percentage
    return target


class Robot:
    def __init__(self):

        # self.left_arm_transport = recoil.SocketCANTransport("can0")
        # self.right_arm_transport = recoil.SocketCANTransport("can1")

        self.left_leg_transport = recoil.SocketCANTransport("can0")
        self.right_leg_transport = recoil.SocketCANTransport("can1")

        # self.left_arm_transport.start()
        # self.right_arm_transport.start()
        self.left_leg_transport.start()
        self.right_leg_transport.start()

        self.joints = [
            # ["left_shoulder_pitch_joint", recoil.MotorController(self.left_arm_transport, 1)],
            # ["left_shoulder_roll_joint", recoil.MotorController(self.left_arm_transport, 3)],
            # ["left_shoulder_yaw_joint", recoil.MotorController(self.left_arm_transport, 5)],
            # ["left_elbow_joint", recoil.MotorController(self.left_arm_transport, 7)],
            # ["left_wrist_yaw_joint", recoil.MotorController(self.left_arm_transport, 9)],

            # ["right_shoulder_pitch_joint", recoil.MotorController(self.right_arm_transport, 2)],
            # ["right_shoulder_roll_joint", recoil.MotorController(self.right_arm_transport, 4)],
            # ["right_shoulder_yaw_joint", recoil.MotorController(self.right_arm_transport, 6)],
            # ["right_elbow_joint", recoil.MotorController(self.right_arm_transport, 8)],
            # ["right_wrist_yaw_joint", recoil.MotorController(self.right_arm_transport, 10)],

            ["left_hip_roll_joint", recoil.MotorController(self.left_leg_transport, 1)],
            ["left_hip_yaw_joint", recoil.MotorController(self.left_leg_transport, 3)],
            ["left_hip_pitch_joint", recoil.MotorController(self.left_leg_transport, 5)],
            ["left_knee_pitch_joint", recoil.MotorController(self.left_leg_transport, 7)],
            ["left_ankle_pitch_joint", recoil.MotorController(self.left_leg_transport, 11)],
            ["left_ankle_roll_joint", recoil.MotorController(self.left_leg_transport, 13)],

            ["right_hip_roll_joint", recoil.MotorController(self.right_leg_transport, 2)],
            ["right_hip_yaw_joint", recoil.MotorController(self.right_leg_transport, 4)],
            ["right_hip_pitch_joint", recoil.MotorController(self.right_leg_transport, 6)],
            ["right_knee_pitch_joint", recoil.MotorController(self.right_leg_transport, 8)],
            ["right_ankle_pitch_joint", recoil.MotorController(self.right_leg_transport, 12)],
            ["right_ankle_roll_joint", recoil.MotorController(self.right_leg_transport, 14)],
        ]

        self.imu = SerialImu(baudrate=Baudrate.BAUD_460800)
        self.imu.run_forever()

        # Start joystick thread
        self.command_controller = Se2Gamepad()
        self.command_controller.run()

        self.state = State.IDLE
        self.next_state = State.IDLE

        self.rl_init_positions = np.array([
            0.0, 0.0, -0.2,
            0.4,
            -0.3, 0.0,
            0.0, 0.0, -0.2,
            0.4,
            -0.3, 0.0
        ], dtype=np.float32)


        self.joint_axis_directions = np.array([
            -1, 1, -1,
            -1,
            -1, 1,
            -1, 1, 1,
            1,
            1, 1
        ], dtype=np.float32)

        self.position_offsets = np.array([
            0.0, 0.0, 0.0,
            0.0,
            0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0,
            0.0, 0.0
        ], dtype=np.float32)

        self.n_lowlevel_states = 4 + 3 + 12 + 12 + 1 + 3
        self.lowlevel_states = np.zeros(self.n_lowlevel_states, dtype=np.float32)

        self.joint_velocity_target = np.zeros(len(self.joints), dtype=np.float32)
        self.joint_position_target = np.zeros(len(self.joints), dtype=np.float32)
        self.joint_position_measured = np.zeros(len(self.joints), dtype=np.float32)
        self.joint_velocity_measured = np.zeros(len(self.joints), dtype=np.float32)

        # used for RL initialization controller
        self.init_percentage = 0.0
        self.starting_positions = np.zeros_like(self.joint_position_target, dtype=np.float32)

        config_path = "calibration.yaml"
        with open(config_path, "r") as f:
            config = OmegaConf.load(f)
        position_offsets = np.array(config.get("position_offsets", None))
        assert position_offsets.shape[0] == len(self.joints)
        self.position_offsets[:] = position_offsets
        

    def run(self):
        self.joint_kp = np.zeros((len(self.joints),), dtype=np.float32)
        self.joint_kd = np.zeros((len(self.joints),), dtype=np.float32)
        self.torque_limit = np.zeros((len(self.joints),), dtype=np.float32)

        self.joint_kp[:] = 20
        self.joint_kd[:] = 2
        self.torque_limit[:] = 1

        for i, entry in enumerate(self.joints):
            joint_name, joint = entry

            print(f"Initializing joint {joint_name}:")
            print(f"  kp: {self.joint_kp[i]}, kd: {self.joint_kd[i]}, torque limit: {self.torque_limit[i]}")

            # Set the joint mode to idle
            joint.set_mode(recoil.Mode.IDLE)
            time.sleep(0.001)
            joint.write_position_kp(self.joint_kp[i])
            time.sleep(0.001)
            joint.write_position_kd(self.joint_kd[i])
            time.sleep(0.001)
            joint.write_torque_limit(self.torque_limit[i])
            time.sleep(0.001)
            joint.feed()
            joint.set_mode(recoil.Mode.DAMPING)

        print("Motors enabled")
            
    def stop(self):
        self.imu.stop()
        self.command_controller.stop()
        
        for entry in self.joints:
            _, joint = entry
            joint.set_mode(recoil.Mode.DAMPING)
        
        print("Entered damping mode. Press Ctrl+C again to exit.\n")

        try:
            while True:
                pass
        except KeyboardInterrupt:
            print("Exiting damping mode.")

        for entry in self.joints:
            _, joint = entry
            joint.set_mode(recoil.Mode.IDLE)

        # self.left_arm_transport.stop()
        # self.right_arm_transport.stop()
        self.left_leg_transport.stop()
        self.right_leg_transport.stop()

    def get_observations(self) -> np.ndarray:
        imu_quaternion = self.lowlevel_states[0:4]
        imu_angular_velocity = self.lowlevel_states[4:7]
        joint_positions = self.lowlevel_states[7:19]
        joint_velocities = self.lowlevel_states[19:31]
        mode = self.lowlevel_states[31:32]
        velocity_commands = self.lowlevel_states[32:35]

        imu_quaternion[:] = self.imu.quaternion[:]

        # IMU returns angular velocity in deg/s, we need rad/s
        imu_angular_velocity[:] = np.deg2rad(self.imu.angular_velocity[:])

        joint_positions[:] = self.joint_position_measured[:]
        joint_velocities[:] = self.joint_velocity_measured[:]

        mode[0] = self.command_controller.commands["mode_switch"]
        velocity_commands[0] = self.command_controller.commands["velocity_x"]
        velocity_commands[1] = self.command_controller.commands["velocity_y"]
        velocity_commands[2] = self.command_controller.commands["velocity_yaw"]
        
        self.next_state = self.command_controller.commands["mode_switch"]

        return self.lowlevel_states

    def update_joints(self):
        position_target = np.zeros_like(self.joint_position_measured, dtype=np.float32)
        # velocity_target = np.zeros_like(self.joint_velocity_measured, dtype=np.float32)
        position_measured = np.zeros_like(self.joint_position_measured, dtype=np.float32)
        velocity_measured = np.zeros_like(self.joint_velocity_measured, dtype=np.float32)

        for i, entry in enumerate(self.joints):
            _, joint = entry
            # adjust direction and offset of target values
            position_target[i] = (self.joint_position_target[i] + self.position_offsets[i]) * self.joint_axis_directions[i]
            # velocity_target = self.joint_velocity_target[i] * self.joint_axis_directions[i]

        # communicate with actuators
        position_measured[0], velocity_measured[0] = self.joints[0][1].write_pdo_2(position_target=position_target[0], velocity_target=0.0)
        position_measured[6], velocity_measured[6] = self.joints[6][1].write_pdo_2(position_target=position_target[6], velocity_target=0.0)

        position_measured[1], velocity_measured[1] = self.joints[1][1].write_pdo_2(position_target=position_target[1], velocity_target=0.0)
        position_measured[7], velocity_measured[7] = self.joints[7][1].write_pdo_2(position_target=position_target[7], velocity_target=0.0)

        position_measured[2], velocity_measured[2] = self.joints[2][1].write_pdo_2(position_target=position_target[2], velocity_target=0.0)
        position_measured[8], velocity_measured[8] = self.joints[8][1].write_pdo_2(position_target=position_target[8], velocity_target=0.0)

        position_measured[3], velocity_measured[3] = self.joints[3][1].write_pdo_2(position_target=position_target[3], velocity_target=0.0)
        position_measured[9], velocity_measured[9] = self.joints[9][1].write_pdo_2(position_target=position_target[9], velocity_target=0.0)

        position_measured[4], velocity_measured[4] = self.joints[4][1].write_pdo_2(position_target=position_target[4], velocity_target=0.0)
        position_measured[10], velocity_measured[10] = self.joints[10][1].write_pdo_2(position_target=position_target[10], velocity_target=0.0)

        position_measured[5], velocity_measured[5] = self.joints[5][1].write_pdo_2(position_target=position_target[5], velocity_target=0.0)
        position_measured[11], velocity_measured[11] = self.joints[11][1].write_pdo_2(position_target=position_target[11], velocity_target=0.0)

        for i, entry in enumerate(self.joints):
            _, joint = entry
            # adjust direction and offset of measured values
            self.joint_position_measured[i] = (position_measured[i] * self.joint_axis_directions[i]) - self.position_offsets[i]
            self.joint_velocity_measured[i] = velocity_measured[i] * self.joint_axis_directions[i]

    def reset(self):
        obs = self.get_observations()
        return obs

    def step(self, actions: np.ndarray):
        """
        actions: np.ndarray of shape (n_joints, )
        """
        match (self.state):
            case State.IDLE:
                self.joint_position_target[:] = self.joint_position_measured[:]

                if self.next_state == State.RL_INIT:
                    print("Switching to RL initialization mode")
                    self.state = self.next_state

                    for entry in self.joints:
                        _, joint = entry
                        joint.feed()
                        joint.set_mode(recoil.Mode.POSITION)

                    self.starting_positions = self.joint_position_target[:]
                    self.init_percentage = 0.0

            case State.RL_INIT:
                print(f"init: {self.init_percentage:.2f}")
                if self.init_percentage < 1.0:
                    self.init_percentage += 1 / 100.0
                    self.init_percentage = min(self.init_percentage, 1.0)
                    
                    for i in range(len(self.joints)):
                        self.joint_position_target[i] = linear_interpolate(self.starting_positions[i], self.rl_init_positions[i], self.init_percentage)
                else:
                    if self.next_state == State.RL_RUNNING:
                        print("Switching to RL running mode")
                        self.state = self.next_state
                    
                    if self.next_state == State.IDLE:
                        print("Switching to idle mode")
                        self.state = self.next_state

                        for entry in self.joints:
                            _, joint = entry
                            joint.set_mode(recoil.Mode.DAMPING)

            case State.RL_RUNNING:
                for i in range(len(self.joints)):
                    self.joint_position_target[i] = actions[i]
                
                if self.next_state == State.IDLE:
                    print("Switching to idle mode")
                    self.state = self.next_state

                    for entry in self.joints:
                        _, joint = entry
                        joint.feed()
                        joint.set_mode(recoil.Mode.DAMPING)

        self.update_joints()

        obs = self.get_observations()

        return obs

    def check_connection(self):
        for entry in self.joints:
            joint_name, joint = entry
            print(f"Pinging {joint_name} ... ", end="\t")
            result = joint.ping()
            if result:
                print("OK")
            else:
                print("ERROR")
            time.sleep(0.1)

ROBOT = Robot()
