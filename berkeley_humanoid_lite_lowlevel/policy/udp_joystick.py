# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import socket
import struct
import time
import threading

from inputs import get_gamepad


class StickEntry:
    AXIS_X_L = "ABS_X"
    AXIS_Y_L = "ABS_Y"
    AXIS_TRIGGER_L = "ABS_Z"
    AXIS_X_R = "ABS_RX"
    AXIS_Y_R = "ABS_RY"
    AXIS_TRIGGER_R = "ABS_RZ"

    BTN_HAT_X = "ABS_HAT0X"
    BTN_HAT_Y = "ABS_HAT0Y"
    
    BTN_A = "BTN_SOUTH"
    BTN_B = "BTN_EAST"
    BTN_X = "BTN_NORTH"
    BTN_Y = "BTN_WEST"
    BTN_BUMPER_L = "BTN_TL"
    BTN_BUMPER_R = "BTN_TR"
    BTN_THUMB_L = "BTN_THUMBL"
    BTN_THUMB_R = "BTN_THUMBR"
    BTN_BACK = "BTN_SELECT"
    BTN_START = "BTN_START"


class UdpController:
    def __init__(self,
                 publish_address: str = "172.28.0.255",
                 publish_port: int = 10011,
                 publish_frequency: float = 20
                 ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.publish_address = (publish_address, publish_port)
        
        self.publish_frequency = publish_frequency

        self.commands = {
            "mode_switch": 0,
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0
        }

        self.stopped = threading.Event()

        print(f"Command controller running at {self.publish_frequency} Hz")

    def run_udp_publish(self) -> None:
        while not self.stopped.is_set():
            # print(f"\r{self.commands}", end='', flush=True)
            udp_data = struct.pack(
                "<Bfff",
                self.commands["mode_switch"],
                self.commands["velocity_x"],
                self.commands["velocity_y"],
                self.commands["velocity_yaw"]
            )
            self.sock.sendto(udp_data, self.publish_address)
            time.sleep(1 / self.publish_frequency)

    def stop(self) -> None:
        print("Joystick stopping...")
        self.stopped.set()
        self.udp_publish_thread.join()
        self.sock.close()

    def run(self) -> None:
        self.udp_publish_thread = threading.Thread(target=self.run_udp_publish)
        self.udp_publish_thread.start()

        self._start_controller()

        while not self.stopped.is_set():
            self._update_controller()

    def _start_controller(self) -> None:
        pass

    def _update_controller(self) -> None:
        pass


class UdpJoystick(UdpController):
    def __init__(self,
                 publish_address: str = "172.28.0.255",
                 publish_port: int = 10011,
                 publish_frequency: float = 20
                 ):
        super().__init__(publish_address, publish_port, publish_frequency)

        self.stick_states = { key: 0 for key in StickEntry.__dict__.values() }

    def _update_controller(self) -> None:
        events = get_gamepad()

        # update all events from the joystick
        for event in events:
            self.stick_states[event.code] = event.state

        velocity_x = self.stick_states.get(StickEntry.AXIS_Y_L)
        velocity_y = self.stick_states.get(StickEntry.AXIS_X_R)
        velocity_yaw = self.stick_states.get(StickEntry.AXIS_X_L)

        if velocity_x is not None:
            self.commands["velocity_x"] = velocity_x / -32768.0
        if velocity_y is not None:
            self.commands["velocity_y"] = velocity_y / -32768.0
        if velocity_yaw is not None:
            self.commands["velocity_yaw"] = velocity_yaw / -32768.0

        mode_switch = 0

        # enter rl control mode
        if self.stick_states.get(StickEntry.BTN_A) and self.stick_states.get(StickEntry.BTN_BUMPER_R):
            mode_switch = 3
        
        # enter init mode
        if self.stick_states.get(StickEntry.BTN_A) and self.stick_states.get(StickEntry.BTN_BUMPER_L):
            mode_switch = 2
        
        # enter idle mode
        if self.stick_states.get(StickEntry.BTN_B) or self.stick_states.get(StickEntry.BTN_THUMB_L) or self.stick_states.get(StickEntry.BTN_THUMB_R):
            mode_switch = 1
        
        self.commands["mode_switch"] = mode_switch    

try:
    from pynput import keyboard
    from pynput.keyboard import Key

    class UdpKeyboard(UdpController):
        def __init__(self,
                    publish_address: str = "172.28.0.255",
                    publish_port: int = 10011,
                    publish_frequency: float = 20
                    ):
            super().__init__(publish_address, publish_port, publish_frequency)

            self.ctrl_pressed = False

        def _on_key_press(self, key: keyboard.Key | keyboard.KeyCode | None) -> None:
            if key is None:
                return

            if not hasattr(key, "char"):
                match key:
                    case Key.ctrl:
                        self.ctrl_pressed = True
                    case Key.space:
                        # request to switch to idle mode
                        self.commands["mode_switch"] = 1
                    case Key.up:
                        self.commands["velocity_x"] += 0.1
                    case Key.down:
                        self.commands["velocity_x"] -= 0.1
                    case Key.left:
                        self.commands["velocity_yaw"] += 0.1
                    case Key.right:
                        self.commands["velocity_yaw"] -= 0.1
                    case Key.end:
                        self.commands["velocity_x"] = 0.0
                        self.commands["velocity_y"] = 0.0
                        self.commands["velocity_yaw"] = 0.0
                return
            
            if type(key) == keyboard.KeyCode:
                if key.char == "2" and self.ctrl_pressed:
                    # request to switch to rl control mode
                    self.commands["mode_switch"] = 3
                if key.char == "1" and self.ctrl_pressed:
                    # request to switch to init mode
                    self.commands["mode_switch"] = 2

        def _on_key_release(self, key: keyboard.Key | keyboard.KeyCode | None) -> None:
            if key is None:
                return

            if not hasattr(key, "char"):
                match key:
                    case Key.ctrl:
                        self.ctrl_pressed = False
                    case Key.space:
                        self.commands["mode_switch"] = 0
                return
            
            if type(key) == keyboard.KeyCode:
                if key.char == "1" or key.char == "2":
                    self.commands["mode_switch"] = 0
                

        def _start_controller(self) -> None:
            self.listener = keyboard.Listener(on_press=self._on_key_press, on_release=self._on_key_release)
            self.listener.start()

except ImportError:
    print("pynput not found, probably on robot computer.")


if __name__ == "__main__":
    command_controller = UdpJoystick()

    try:
        command_controller.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    
    command_controller.stop()
