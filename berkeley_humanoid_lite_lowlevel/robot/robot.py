# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time

import numpy as np

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.robot.imu import SerialImu, Baudrate


class State:
    IDLE = 0
    RL_INIT = 1
    RL_RUNNING = 2


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

        # Initialize buffers
        # observations:
        # [0:4]: imu quaternion (w, x, y, z)
        # [4:7]: imu angular velocity (rx, ry, rz)
        # [7:19]: joint positions
        # [19:31]: joint velocities
        # [31:32]: mode
        # [32:35]: velocity commands
        self.n_lowlevel_states = 4 + 3 + 12 + 12 + 1 + 3
        self.lowlevel_states = np.zeros(self.n_lowlevel_states, dtype=np.float32)

        self.joint_velocity_target = np.zeros(len(self.joints), dtype=np.float32)
        self.joint_position_target = np.zeros(len(self.joints), dtype=np.float32)
        self.joint_position_measured = np.zeros(len(self.joints), dtype=np.float32)
        self.joint_velocity_measured = np.zeros(len(self.joints), dtype=np.float32)
        
    def stop(self):

        self.imu.stop()
        
        for joint in self.joints:
            joint.set_mode(recoil.Mode.DAMPING)
        
        print("Entered damping mode. Press Ctrl+C again to exit.\n")


        try:
            while True:
                pass
        except KeyboardInterrupt:
            print("Exiting damping mode.")

        for joint in self.joints:
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
        imu_angular_velocity[:] = self.imu.angular_velocity[:]
        joint_positions[:] = self.joint_position_measured[:]
        joint_velocities[:] = self.joint_velocity_measured[:]
        mode[0] = 0.0
        velocity_commands[:] = 0.0
        return self.lowlevel_states

    def update_joints(self):
        for i, entry in enumerate(self.joints):
            joint_name, joint = entry

            # adjust direction and offset of target values
            position_target = (self.joint_position_target[i] + self.position_offsets[i]) * self.joint_axis_directions[i]
            velocity_target = self.joint_velocity_target[i] * self.joint_axis_directions[i]

            # communicate with actuators
            position_measured, velocity_measured = joint.write_pdo_2(position_target=position_target, velocity_target=0.0)

            # adjust direction and offset of measured values
            self.joint_position_measured[i] = (position_measured * self.joint_axis_directions[i]) - self.position_offsets[i]
            self.joint_velocity_measured[i] = velocity_measured * self.joint_axis_directions[i]

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

                    for joint in self.joints:
                        joint.feed()
                        joint.set_mode(recoil.Mode.POSITION)
                    
                    starting_positions = self.joint_position_target[:]
                    init_percentage = 0.0

            case State.RL_INIT:
                if init_percentage < 1.0:
                    init_percentage += 1 / 200.0
                    init_percentage = min(init_percentage, 1.0)
                    
                    for i in range(len(self.joints)):
                        self.joint_position_target[i] = linear_interpolate(starting_positions[i], self.rl_init_positions[i], init_percentage)
                else:
                    if self.next_state == State.RL_RUNNING:
                        print("Switching to RL running mode")
                        self.state = self.next_state
                    
                    if self.next_state == State.IDLE:
                        print("Switching to idle mode")
                        self.state = self.next_state

                        for joint in self.joints:
                            joint.set_mode(recoil.Mode.DAMPING)

            case State.RL_RUNNING:
                # for (int i = 0; i < N_JOINTS; i += 1) {
                #     position_target[i] = lowlevel_commands[i];
                # }
                # if (next_state == STATE_IDLE) {
                #     printf("Switching to idle mode\n");
                #     state = next_state;

                #     for (int i = 0; i < N_JOINTS; i += 1) {
                #         usleep(5);
                #         joint_ptrs[i]->set_mode(MODE_DAMPING);
                #     }
                # }
                pass

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
