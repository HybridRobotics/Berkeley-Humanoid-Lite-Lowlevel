# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse
from typing import Union

import numpy as np
from cc.udp import UDP
from omegaconf import DictConfig, ListConfig, OmegaConf
from loop_rate_limiters import RateLimiter

from berkeley_humanoid_lite_lowlevel.robot import ROBOT
from berkeley_humanoid_lite_lowlevel.policy.rl_controller import RlController
from berkeley_humanoid_lite_lowlevel.policy.config import Cfg


# Load configuration
cfg = Cfg.from_arguments()

print(f"Policy frequency: {1 / cfg.policy_dt} Hz")

udp = UDP(("0.0.0.0", 11000), ("172.28.0.5", 11000))


# Initialize and start policy controller
controller = RlController(cfg)
controller.load_policy()


rate = RateLimiter(1 / cfg.policy_dt)

ROBOT.run()

obs = ROBOT.reset()

try:
    while True:
        actions = controller.update(obs)
        obs = ROBOT.step(actions)
        udp.send_numpy(obs)

        rate.sleep()

except KeyboardInterrupt:
    ROBOT.stop()

print("Stopped.")
