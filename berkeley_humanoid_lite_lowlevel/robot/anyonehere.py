"""
anyonehere.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

This script will send a message to all devices on the CAN bus and print the
response. Useful for detecting new devices on the can bus and checking the
connectivity between the lowlevel computer and the joint controllers.
"""

import argparse

import berkeley_humanoid_lite_lowlevel.recoil as recoil


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", help="CAN transport channel", type=str, default="can0")
args = parser.parse_args()

bus = recoil.Bus(channel=args.port, baudrate=1000000)

for i in range(16):
    bus.ping(i)

bus.stop()
