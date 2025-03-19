# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time

import numpy as np

import berkeley_humanoid_lite_lowlevel.recoil as recoil


class Robot:
    def __init__(self, enable_arms: bool = False, enable_legs: bool = False):
        self.left_arm_transport = None
        self.right_arm_transport = None
        self.left_leg_transport = None
        self.right_leg_transport = None

        if enable_arms:
            self.left_arm_transport = recoil.SocketCANTransport("can0")
            self.right_arm_transport = recoil.SocketCANTransport("can1")

        if enable_legs:
            self.left_leg_transport = recoil.SocketCANTransport("can3")
            self.right_leg_transport = recoil.SocketCANTransport("can2")

        if enable_arms:
            self.left_arm_transport.start()
            self.right_arm_transport.start()

        if enable_legs:
            self.left_leg_transport.start()
            self.right_leg_transport.start()

        joints = []
        joint_directions = []

        if enable_arms:
            joints += [
                ["left_shoulder_pitch_joint", recoil.MotorController(self.left_arm_transport, 1)],
                ["left_shoulder_roll_joint", recoil.MotorController(self.left_arm_transport, 3)],
                ["left_shoulder_yaw_joint", recoil.MotorController(self.left_arm_transport, 5)],
                ["left_elbow_joint", recoil.MotorController(self.left_arm_transport, 7)],
                ["left_wrist_yaw_joint", recoil.MotorController(self.left_arm_transport, 9)],

                ["right_shoulder_pitch_joint", recoil.MotorController(self.right_arm_transport, 2)],
                ["right_shoulder_roll_joint", recoil.MotorController(self.right_arm_transport, 4)],
                ["right_shoulder_yaw_joint", recoil.MotorController(self.right_arm_transport, 6)],
                ["right_elbow_joint", recoil.MotorController(self.right_arm_transport, 8)],
                ["right_wrist_yaw_joint", recoil.MotorController(self.right_arm_transport, 10)],
            ]
            joint_directions += [
                 1,  1, -1, -1, -1,
                -1,  1, -1,  1, -1,
            ]

        if enable_legs:
            joints += [
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
            joint_directions += [
                -1,  1, -1, -1, -1,  1,
                -1,  1,  1,  1,  1,  1,
            ]
        self.joints = [joint for joint in joints]
        self.joint_directions = np.array(joint_directions)

        n_joints = len(joints)
        self.position_target = np.zeros(n_joints)
        self.position_measured = np.zeros(n_joints)
        self.position_offset = np.zeros(n_joints)

    def stop(self):
        if self.left_arm_transport:
            self.left_arm_transport.stop()
        if self.right_arm_transport:
            self.right_arm_transport.stop()
        if self.left_leg_transport:
            self.left_leg_transport.stop()
        if self.right_leg_transport:
            self.right_leg_transport.stop()

    def update_offset(self):
        for i, entry in enumerate(self.joints):
            _, joint = entry
            self.position_offset[i] = joint.read_position_measured() * self.joint_directions[i]
    
    def update_joint_measurements(self) -> np.ndarray:
        for i, entry in enumerate(self.joints):
            _, joint = entry
            self.position_measured[i] = joint.read_position_measured() * self.joint_directions[i] - self.position_offset[i]
        return self.position_measured

    def update_joint_targets(self):
        for i, entry in enumerate(self.joints):
            _, joint = entry
            joint.write_position_target((self.position_target[i] + self.position_offset[i]) * self.joint_directions[i])

    def update(self) -> np.ndarray:
        self.update_joint_targets()
        self.update_joint_measurements()
        for i, entry in enumerate(self.joints):
            _, joint = entry
            joint.feed()
        return self.position_measured

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
