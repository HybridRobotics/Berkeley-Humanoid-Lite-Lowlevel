# Copyright (c) 2025, -T.K.-.

import os
import struct

import can
import serial

from .core import DataFrame, Transport, Function


class CANFrame(DataFrame):
    ID_STANDARD = 0
    ID_EXTENDED = 1

    DEVICE_ID_MSK = 0x7F
    FUNC_ID_POS = 7
    FUNC_ID_MSK = 0x0F << FUNC_ID_POS

    def __init__(self, 
            device_id: int = 0, 
            func_id: Function = None, 
            size: int = 0, 
            data: bytes = b""
        ):
        super().__init__(device_id, func_id, size, data)
        assert self.size <= 8


class CANTransport(Transport):
    def __init__(self, port, baudrate=1000000):
        super().__init__(port)
        self.port = port
        self.baudrate = baudrate
        self._interface = None
        self._handlers = []

    def stop(self):
        self._killed.set()
        if self._interface:
            self._interface.shutdown()

    def start(self):
        self._killed.clear()
        self.connect()
        print("started")

    def connect(self):
        while not self._interface:
            try:
                self._interface = can.Bus(interface=self.INTERFACE, channel=self.port, baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                print(e)
        print("connected")

    """
    Receive data.

    timeout == None: blocking forever
    timeout == 0: non-blocking (the actual delay is around 0.1s)
    timeout > 0: blocking for timeout seconds

    @param timeout: timeout in seconds
    """
    def receive(self, 
                match_function: Function = None,
                match_device_id: int = None,
                timeout=None
                ) -> CANFrame:
        while True:
            try:
                msg = self._interface.recv(timeout=timeout)
            except can.exceptions.CanOperationError as e:
                print("<CANReceive> error:", e)
                return None
            except TypeError as e:
                print("<CANReceive> error:", e)
                return None
            
            if not msg:
                return None

            if msg.is_error_frame:
                print("<CANReceive> WARNING: received Error Frame")
                continue
            
            frame = CANFrame(
                device_id = msg.arbitration_id & CANFrame.DEVICE_ID_MSK,
                func_id = msg.arbitration_id >> CANFrame.FUNC_ID_POS,
                size = msg.dlc,
                data = msg.data
            )
            if match_function:
                if frame.func_id != match_function:
                    continue
            if match_device_id:
                if frame.device_id != match_device_id:
                    continue
            
            return frame

    def transmit(self, frame: CANFrame):
        assert frame.device_id <= CANFrame.DEVICE_ID_MSK, "device_id: {0} out of range".format(frame.device_id)
        
        can_id = (frame.func_id << CANFrame.FUNC_ID_POS) | frame.device_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            data=frame.data)

        self._interface.send(msg)
    
    def anyone(self, id_range=CANFrame.DEVICE_ID_MSK):
        for i in range(1, id_range+1):
            print(f"testing id {i}... \t", end="")
            tx_frame = CANFrame(i, Function.RECEIVE_PDO_1, size=1, data=struct.pack("<B", 0xCA))
            self.transmit(tx_frame)
            rx_frame = self.receive(timeout=0.2)
            if not rx_frame:
                print("FAILED: no response")
                continue
            if rx_frame.func_id != Function.TRANSMIT_PDO_1 or rx_frame.device_id != i or rx_frame.data[0] != 0xCA:
                print("FAILED: wrong response")
                continue
            print(f"SUCCESS: {i}")


class SPICANTransport(CANTransport):
    INTERFACE = "socketcan"

    def stop(self):
        self._killed.set()
        os.system("sudo ifconfig {port} down".format(port=self.port))

    def start(self):
        os.system("sudo ip link set {port} type can bitrate {baudrate}".format(port=self.port, baudrate=self.baudrate))
        os.system("sudo ifconfig {port} up".format(port=self.port))

        self._killed.clear()
        self.connect()

        print("started")


class SerialCANTransport(CANTransport):
    INTERFACE= "serial"

class SocketCANTransport(CANTransport):
    """
    This is a socketcan transport that uses the socketcan interface.

    To enable socketcan, run the following terminal command:

    ```bash
    sudo ip link set can0 up type can bitrate 1000000
    ```

    To disable socketcan, run the following terminal command:

    ```bash
    sudo ip link set can0 down
    ```

    Some other useful commands:

    ```bash
    candump can0  # monitor can0 messages

    cansend can0 201#1122334455667788  # send a ping message
    ```
    """
    INTERFACE = "socketcan"

    def stop(self):
        self._killed.set()
        self._interface.shutdown()

    def start(self):
        self._killed.clear()
        self.connect()

        print("started")

