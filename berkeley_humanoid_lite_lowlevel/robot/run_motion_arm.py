import time

import numpy as np
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
import matplotlib.pyplot as plt
from cc.udp import UDP

from arm import Arm

# configure float print precision
np.set_printoptions(precision=3)


# Initialize UDP with both send and receive addresses
udp = UDP(
    recv_addr=("0.0.0.0", 8000),  # Listen on all interfaces
    send_addr=("172.28.0.7", 11012)
)

motion = np.load("motions/motion.npy")

motion *= -1.0


# plt.plot(motion, label=["j0", "j1", "j2", "j3", "j4"])
# plt.legend()
# plt.show()


robot = Arm()

rate = RateLimiter(frequency=50.0)

robot.check_connection()

robot.update_offset()
robot.update()

time.sleep(0.1)

for entry in robot.joints:
    _, joint = entry
    joint.write_torque_limit(6)

time.sleep(0.1)

for entry in robot.joints:
    _, joint = entry
    joint.set_mode(recoil.Mode.POSITION)
    joint.feed()

try:
    for run in range(1):
        print(f"Run {run} / 20")
        for i in range(motion.shape[0]):
            robot.position_target[:] = motion[i, :]
            robot.update()

            # print()
            print(i, robot.position_measured[:])

            if i % 300 == 175:
                udp.send_numpy(np.array([1.0], dtype=np.float32))



            rate.sleep()
        


except KeyboardInterrupt:
    print("Interrupted")

print("Done.")

for entry in robot.joints:
    _, joint = entry
    joint.set_mode(recoil.Mode.DAMPING)

input("Press Enter to stop...")

for entry in robot.joints:
    _, joint = entry
    joint.set_mode(recoil.Mode.IDLE)

print("Stopping...")

robot.stop()



