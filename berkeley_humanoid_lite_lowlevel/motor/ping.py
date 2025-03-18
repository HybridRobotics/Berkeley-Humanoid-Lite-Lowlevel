# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
transport = recoil.SocketCANTransport(port=args.port, baudrate=1000000)
transport.start()

motor = recoil.MotorController(transport, id=args.id)

status = motor.ping()

if status:
    print("Motor is online")
else:
    print("Motor is offline")

transport.stop()
