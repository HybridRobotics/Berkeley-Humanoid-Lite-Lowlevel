import time

import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
transport = recoil.SocketCANTransport(port=args.port, baudrate=1000000)
transport.start()

motor = recoil.MotorController(transport, id=args.id)

print("max calibration current:", motor._read_parameter_f32(recoil.Parameter.MOTOR_MAX_CALIBRATION_CURRENT))

print("starting calibration")
motor.set_mode(recoil.Mode.CALIBRATION)
time.sleep(5)

for i in range(10):
    print("calibrating...")
    time.sleep(1)

print("calibration offset:", motor._read_parameter_f32(recoil.Parameter.ENCODER_FLUX_OFFSET))

motor.set_mode(recoil.Mode.IDLE)

transport.stop()
