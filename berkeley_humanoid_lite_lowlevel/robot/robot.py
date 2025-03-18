# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time

import berkeley_humanoid_lite_lowlevel.recoil as recoil


class Robot:
    def __init__(self):

        self.left_arm_transport = recoil.SocketCANTransport("can0")
        self.right_arm_transport = recoil.SocketCANTransport("can1")

        self.left_leg_transport = recoil.SocketCANTransport("can3")
        self.right_leg_transport = recoil.SocketCANTransport("can2")

        self.left_arm_transport.start()
        self.right_arm_transport.start()
        self.left_leg_transport.start()
        self.right_leg_transport.start()

        self.joints = [
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

    def stop(self):
        self.left_arm_transport.stop()
        self.right_arm_transport.stop()
        self.left_leg_transport.stop()
        self.right_leg_transport.stop()

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