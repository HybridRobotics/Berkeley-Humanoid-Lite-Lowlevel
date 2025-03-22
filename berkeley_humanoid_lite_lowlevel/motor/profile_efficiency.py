import time
import json
import threading

import serial
import numpy as np
import matplotlib.pyplot as plt

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.policy import KeyboardController


args = recoil.util.get_args()
transport = recoil.SocketCANTransport(port=args.port, baudrate=1000000)
transport.start()

motor = recoil.MotorController(transport, id=args.id)

keycontroller = KeyboardController()


print("connecting to device #{id}...".format(id=args.id))


stopped = threading.Event()

sensor_started = threading.Event()


moteus_velocity_target = 1.


monitor_data = {
    "torque": 0.,
    "velocity": moteus_velocity_target,
    "voltage": 0.,
    "current": 0.,
}

print(f"Experiment with velocity = {moteus_velocity_target}")


def receive_monitor():
    global monitor_data

    arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)

    # wait for sensor to initialize
    for i in range(5):
        print(f"waiting for sensor to initialize... {i+1} / 5...")
        time.sleep(1)
    
    sensor_started.set()

    while not stopped.is_set():
        buffer = arduino.readline()
        try:
            data = json.loads(buffer)
        except (json.decoder.JSONDecodeError, UnicodeDecodeError):
            print("failed to decode json:")
            print(buffer)
            continue


        monitor_data["torque"] = data["T"]
        monitor_data["voltage"] = data["V"]
        monitor_data["current"] = data["I"]
    
    arduino.close()



monitor_thread = threading.Thread(target=receive_monitor)

monitor_thread.start()

# wait for sensor to initialize
while not sensor_started.is_set():
    time.sleep(1)

motor.write_gear_ratio(-15.)

time.sleep(0.1)

motor.write_torque_limit(20)
motor.write_position_kp(10)
motor.write_velocity_kp(1)


time.sleep(0.1)


input("Press Enter to start...")

print("starting keyboard controller...")

keycontroller.start()

motor.write_torque_target(0)
motor.set_mode(recoil.Mode.TORQUE)

torque_start = 0
torque_end = 20
n_steps = 50

recorded_data = []

try:
    for torque in np.linspace(torque_start, torque_end, n_steps+1):
        print(f"Testing torque = {torque} Nm...")
        motor.write_torque_target(torque)

        # wait for sensor to stabilize
        for i in range(10):
            try:
                motor.feed()
            except OSError as e:
                print(f"Error feeding motor: {e}")
            time.sleep(0.1)
        
        motor_torque = motor.read_torque_measured()
        
        data_entry = [torque, motor_torque, monitor_data["torque"], monitor_data["velocity"], monitor_data["voltage"], monitor_data["current"]]
        recorded_data.append(data_entry)

        if keycontroller.get("Q") or keycontroller.get("X"):
            print("stopping program...")
            break

except KeyboardInterrupt:
    print("stopping program...")


recorded_data = np.array(recorded_data)


np.savetxt(f"torque_sweep_{moteus_velocity_target}_rad_s.csv", recorded_data, delimiter=",", header="torque_target,torque_measured_motor,torque_measured_loadcell,velocity,voltage,current")


stopped.set()
motor.set_mode(recoil.Mode.IDLE)
transport.stop()


mechanical_power = recorded_data[:, 2] * recorded_data[:, 3]
electrical_power = recorded_data[:, 4] * recorded_data[:, 5]


fig, (ax1, ax2) = plt.subplots(2, 1)

# Measured torque vs commanded torque
ax1.plot(recorded_data[:, 0], recorded_data[:, 1], label="measured")
ax1.plot(recorded_data[:, 0], recorded_data[:, 0], label="commanded")
ax1.legend()
ax1.set_xlabel("Commanded torque [Nm]")
ax1.set_ylabel("Measured torque [Nm]")

# mechanical power vs electrical power ratio
ax2.plot(recorded_data[:, 0], mechanical_power / electrical_power)
ax2.set_xlabel("Commanded torque [Nm]")
ax2.set_ylabel("Mechanical power / Electrical power")

plt.show()

