# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from berkeley_humanoid_lite_lowlevel.robot.robot import Robot


robot = Robot()

robot.check_connection()

robot.stop()
