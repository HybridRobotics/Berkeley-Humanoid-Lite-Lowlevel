# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time

import numpy as np

import berkeley_humanoid_lite_lowlevel.recoil as recoil


class Arm:
    def __init__(self):
        self.transport = recoil.SocketCANTransport("can0")
        self.transport.start()

        self.joints = [
            ["left_shoulder_pitch_joint", recoil.MotorController(self.transport, 1)],
            ["left_shoulder_roll_joint", recoil.MotorController(self.transport, 3)],
            ["left_shoulder_yaw_joint", recoil.MotorController(self.transport, 5)],
            ["left_elbow_joint", recoil.MotorController(self.transport, 7)],
            ["left_wrist_yaw_joint", recoil.MotorController(self.transport, 9)],
        ]


        self.joint_directions = np.array([1,  1, -1, -1, -1])

        n_joints = len(self.joints)
        self.position_target = np.zeros(n_joints)
        self.position_measured = np.zeros(n_joints)
        self.position_offset = np.zeros(n_joints)

    def stop(self):
        self.transport.stop()
        
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
