import threading

from loop_rate_limiters import RateLimiter
import numpy as np
from cc.udp import UDP
import serial
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# configure float print precision
np.set_printoptions(precision=3)


udp = UDP(recv_addr=("172.28.0.2", 10100), send_addr=("172.28.0.101", 10101))

stopped = threading.Event()
udp_timeout = threading.Event()

udp_received_data = np.zeros(12)



def udp_thread():
    global udp, udp_received_data

    while not stopped.is_set():
        buffer = udp.recv_numpy(dtype=np.float32, timeout=0.1)
        if buffer is not None:
            udp_received_data[:] = buffer
        else:
            udp_timeout.set()
        

udp_thread = threading.Thread(target=udp_thread)
udp_thread.start()

ser = serial.Serial("/dev/ttyUSB0", 115200)


transport_arm_left = recoil.SocketCANTransport(port="can0")
transport_arm_right = recoil.SocketCANTransport(port="can1")
transport_arm_left.start()
transport_arm_right.start()

ids_arm_left = [1, 3, 5, 7, 9]
ids_arm_right = [2, 4, 6, 8, 10]

joints = []

joints += [
    recoil.MotorController(transport_arm_left, id=id) for id in ids_arm_left
]
joints += [
    recoil.MotorController(transport_arm_right, id=id) for id in ids_arm_right
]

n_joints = 10

joint_directions = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1], dtype=np.float32)

joint_offsets = np.zeros(n_joints, dtype=np.float32)
joint_position_target = np.zeros(n_joints, dtype=np.float32)
joint_position_measured = np.zeros(n_joints, dtype=np.float32)


for i, joint in enumerate(joints):
    print(joint.id)
    joint_offsets[i] = joint.read_position_measured()



# shoulder should be incremented by 90 degrees to match URDF
joint_offsets[1] += np.pi / 2
joint_offsets[6] -= np.pi / 2

for i, joint in enumerate(joints):
    joint.write_torque_limit(4)

    # initialize joint_position_measured with current position
    joint_position_measured[i] = joint.read_position_measured() * joint_directions[i] - joint_offsets[i]

# initialize udp_received_data with current position
udp_received_data[0:10] = joint_position_measured

rate = RateLimiter(frequency=50.0)


for joint in joints:
    # pass
    # joint.set_mode(recoil.Mode.POSITION)
    joint.feed()

try:
    while True:

        if udp_timeout.is_set():
            print("No data received, using current position")
            joint_position_target[:] = joint_position_measured
            gripper_l_command = b"0"
            gripper_r_command = b"a"
        else:
            joint_position_target[:] = udp_received_data[0:10]
            gripper_left_float = udp_received_data[10]
            gripper_right_float = udp_received_data[11]
            if gripper_left_float < 0.1:
                gripper_l_command = b"0"
            elif gripper_left_float < 0.2:
                gripper_l_command = b"1"
            elif gripper_left_float < 0.3:
                gripper_l_command = b"2"
            elif gripper_left_float < 0.4:
                gripper_l_command = b"3"
            elif gripper_left_float < 0.5:
                gripper_l_command = b"4"
            elif gripper_left_float < 0.6:
                gripper_l_command = b"5"
            elif gripper_left_float < 0.7:
                gripper_l_command = b"6"
            elif gripper_left_float < 0.8:
                gripper_l_command = b"7"
            elif gripper_left_float < 0.9:
                gripper_l_command = b"8"
            else:
                gripper_l_command = b"9"

            if gripper_right_float < 0.1:
                gripper_r_command = b"a"
            elif gripper_right_float < 0.2:
                gripper_r_command = b"b"
            elif gripper_right_float < 0.3:
                gripper_r_command = b"c"
            elif gripper_right_float < 0.4:
                gripper_r_command = b"d"
            elif gripper_right_float < 0.5:
                gripper_r_command = b"e"
            elif gripper_right_float < 0.6:
                gripper_r_command = b"f"
            elif gripper_right_float < 0.7:
                gripper_r_command = b"g"
            elif gripper_right_float < 0.8:
                gripper_r_command = b"h"
            elif gripper_right_float < 0.9:
                gripper_r_command = b"i"
            else:
                gripper_r_command = b"j"


        for i, joint in enumerate(joints):
            joint.write_position_target((joint_position_target[i] + joint_offsets[i]) * joint_directions[i])
            joint.feed()
            joint_position_measured[i] = joint.read_position_measured() * joint_directions[i] - joint_offsets[i]
        
        udp.send_numpy(joint_position_measured)



        ser.write(gripper_l_command)
        ser.write(gripper_r_command)

        # print(joint_position_measured, joint_position_target)
        # print(udp_received_data[10], udp_received_data[11])
        
        rate.sleep()

except KeyboardInterrupt:
    print("Interrupted")

for joint in joints:
    joint.set_mode(recoil.Mode.DAMPING)

input("Press Enter to stop...")

for joint in joints:
    joint.set_mode(recoil.Mode.IDLE)

stopped.set()

transport_arm_left.stop()
transport_arm_right.stop()



