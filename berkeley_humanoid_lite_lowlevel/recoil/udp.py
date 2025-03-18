# Copyright (c) 2025, -T.K.-.

import struct

from cc.udp import UDP, UDP

from .core import DataFrame, Transport


class UDPTransport(Transport):
    def __init__(self, rx_udp: UDP, tx_udp: UDP):
        super().__init__(rx_udp.addr)
        self.rx_addr = rx_udp.addr
        self.tx_addr = tx_udp.addr
        self.rx_udp = rx_udp
        self.tx_udp = tx_udp
        
    def stop(self):
        self.rx_udp.stop()
        self.tx_udp.stop()

    def start(self):
        self._killed.clear()
        print("started")

    """
    Receive data.

    timeout == None: blocking forever
    timeout == 0: non-blocking (the actual delay is around 0.1s)
    timeout > 0: blocking for timeout seconds

    @param timeout: timeout in seconds
    """
    def receive(self, 
                match_frame: DataFrame = None,
                timeout=None
                ) -> DataFrame:
        while True:
            buffer = self.rx_udp.recv(timeout=timeout)
            
            if not buffer:
                return None
            
            device_id, func_id, size = struct.unpack("<HHL", buffer[:8])
            data = buffer[8:]
            frame = DataFrame(device_id, func_id, size, data)

            if match_frame:
                if frame.device_id != match_frame.device_id or frame.func_id != match_frame.func_id:
                    continue
            
            return frame
        
    def transmit(self, frame: DataFrame):
        header = struct.pack("<HHL", frame.device_id, frame.func_id, frame.size)
        buffer = header + frame.data
        self.tx_udp.send(buffer)

