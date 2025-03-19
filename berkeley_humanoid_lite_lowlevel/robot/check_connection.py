# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from robot import Robot


robot = Robot(enable_arms=True, enable_legs=True)

robot.check_connection()

robot.stop()

