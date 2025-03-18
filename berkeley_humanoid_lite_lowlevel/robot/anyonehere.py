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
parser.add_argument("-p", "--port", help="CAN port", type=str, default="can0")
args = parser.parse_args()

transport = recoil.SocketCANTransport(port=args.port, baudrate=1000000)
transport.start()

transport.anyone(16)

transport.stop()
