import time

from loop_rate_limiters import RateLimiter
import numpy as np
import berkeley_humanoid_lite_lowlevel.recoil as recoil

from robot import Robot

# configure float print precision
np.set_printoptions(precision=3)


robot = Robot(enable_arms=True, enable_legs=False)

rate = RateLimiter(frequency=50.0)

robot.check_connection()

robot.update_offset()
robot.update()

time.sleep(0.1)

for entry in robot.joints:
    _, joint = entry
    joint.write_torque_limit(1)

time.sleep(0.1)

for entry in robot.joints:
    _, joint = entry
    joint.set_mode(recoil.Mode.POSITION)
    joint.feed()


try:
    while True:
        robot.position_target[3] = -0.0
        robot.update()

        print(robot.position_measured[3])

        rate.sleep()


except KeyboardInterrupt:
    print("Interrupted")

for entry in robot.joints:
    _, joint = entry
    joint.set_mode(recoil.Mode.DAMPING)

input("Press Enter to stop...")

for entry in robot.joints:
    _, joint = entry
    joint.set_mode(recoil.Mode.IDLE)

robot.stop()



