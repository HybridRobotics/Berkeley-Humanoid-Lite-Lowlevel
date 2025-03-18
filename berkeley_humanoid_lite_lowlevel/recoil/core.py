# Copyright (c) 2025, -T.K.-.

import math
import struct
import threading


class Function:
    NMT                             = 0b0000
    SYNC_EMCY                       = 0b0001
    TIME                            = 0b0010
    TRANSMIT_PDO_1                  = 0b0011
    RECEIVE_PDO_1                   = 0b0100
    TRANSMIT_PDO_2                  = 0b0101
    RECEIVE_PDO_2                   = 0b0110
    TRANSMIT_PDO_3                  = 0b0111
    RECEIVE_PDO_3                   = 0b1000
    TRANSMIT_PDO_4                  = 0b1001
    RECEIVE_PDO_4                   = 0b1010
    TRANSMIT_SDO                    = 0b1011
    RECEIVE_SDO                     = 0b1100
    FLASH                           = 0b1101
    HEARTBEAT                       = 0b1110


class Mode:
    # these are three safe modes
    DISABLED                        = 0x00
    IDLE                            = 0x01

    # these are special modes
    DAMPING                         = 0x02
    CALIBRATION                     = 0x05

    # these are closed-loop modes
    CURRENT                         = 0x10
    TORQUE                          = 0x11
    VELOCITY                        = 0x12
    POSITION                        = 0x13

    # these are open-loop modes
    VABC_OVERRIDE                   = 0x20
    VALPHABETA_OVERRIDE             = 0x21
    VQD_OVERRIDE                    = 0x22

    DEBUG                           = 0x80


class ErrorCode:
    NO_ERROR                        = 0b0000000000000000
    GENERAL                         = 0b0000000000000001
    ESTOP                           = 0b0000000000000010
    INITIALIZATION_ERROR            = 0b0000000000000100
    CALIBRATION_ERROR               = 0b0000000000001000
    POWERSTAGE_ERROR                = 0b0000000000010000
    INVALID_MODE                    = 0b0000000000100000
    WATCHDOG_TIMEOUT                = 0b0000000001000000
    OVER_VOLTAGE                    = 0b0000000010000000
    OVER_CURRENT                    = 0b0000000100000000
    OVER_TEMPERATURE                = 0b0000001000000000
    CAN_TX_FAULT                    = 0b0000010000000000
    I2C_FAULT                       = 0b0000100000000000


# supported version: >= 1.1.1
class Parameter:
    DEVICE_ID                                       = 0x000
    FIRMWARE_VERSION                                = 0x004
    WATCHDOG_TIMEOUT                                = 0x008
    FAST_FRAME_FREQUENCY                            = 0x00C
    MODE                                            = 0x010
    ERROR                                           = 0x014
    POSITION_CONTROLLER_UPDATE_COUNTER              = 0x018
    POSITION_CONTROLLER_GEAR_RATIO                  = 0x01C
    POSITION_CONTROLLER_POSITION_KP                 = 0x020
    POSITION_CONTROLLER_POSITION_KI                 = 0x024
    POSITION_CONTROLLER_VELOCITY_KP                 = 0x028
    POSITION_CONTROLLER_VELOCITY_KI                 = 0x02C
    POSITION_CONTROLLER_TORQUE_LIMIT                = 0x030
    POSITION_CONTROLLER_VELOCITY_LIMIT              = 0x034
    POSITION_CONTROLLER_POSITION_LIMIT_LOWER        = 0x038
    POSITION_CONTROLLER_POSITION_LIMIT_UPPER        = 0x03C
    POSITION_CONTROLLER_POSITION_OFFSET             = 0x040
    POSITION_CONTROLLER_TORQUE_TARGET               = 0x044
    POSITION_CONTROLLER_TORQUE_MEASURED             = 0x048
    POSITION_CONTROLLER_TORQUE_SETPOINT             = 0x04C
    POSITION_CONTROLLER_VELOCITY_TARGET             = 0x050
    POSITION_CONTROLLER_VELOCITY_MEASURED           = 0x054
    POSITION_CONTROLLER_VELOCITY_SETPOINT           = 0x058
    POSITION_CONTROLLER_POSITION_TARGET             = 0x05C
    POSITION_CONTROLLER_POSITION_MEASURED           = 0x060
    POSITION_CONTROLLER_POSITION_SETPOINT           = 0x064
    POSITION_CONTROLLER_POSITION_INTEGRATOR         = 0x068
    POSITION_CONTROLLER_VELOCITY_INTEGRATOR         = 0x06C
    POSITION_CONTROLLER_TORQUE_FILTER_ALPHA         = 0x070
    CURRENT_CONTROLLER_I_LIMIT                      = 0x074
    CURRENT_CONTROLLER_I_KP                         = 0x078
    CURRENT_CONTROLLER_I_KI                         = 0x07C
    CURRENT_CONTROLLER_I_A_MEASURED                 = 0x080
    CURRENT_CONTROLLER_I_B_MEASURED                 = 0x084
    CURRENT_CONTROLLER_I_C_MEASURED                 = 0x088
    CURRENT_CONTROLLER_V_A_SETPOINT                 = 0x08C
    CURRENT_CONTROLLER_V_B_SETPOINT                 = 0x090
    CURRENT_CONTROLLER_V_C_SETPOINT                 = 0x094
    CURRENT_CONTROLLER_I_ALPHA_MEASURED             = 0x098
    CURRENT_CONTROLLER_I_BETA_MEASURED              = 0x09C
    CURRENT_CONTROLLER_V_ALPHA_SETPOINT             = 0x0A0
    CURRENT_CONTROLLER_V_BETA_SETPOINT              = 0x0A4
    CURRENT_CONTROLLER_V_Q_TARGET                   = 0x0A8
    CURRENT_CONTROLLER_V_D_TARGET                   = 0x0AC
    CURRENT_CONTROLLER_V_Q_SETPOINT                 = 0x0B0
    CURRENT_CONTROLLER_V_D_SETPOINT                 = 0x0B4
    CURRENT_CONTROLLER_I_Q_TARGET                   = 0x0B8
    CURRENT_CONTROLLER_I_D_TARGET                   = 0x0BC
    CURRENT_CONTROLLER_I_Q_MEASURED                 = 0x0C0
    CURRENT_CONTROLLER_I_D_MEASURED                 = 0x0C4
    CURRENT_CONTROLLER_I_Q_SETPOINT                 = 0x0C8
    CURRENT_CONTROLLER_I_D_SETPOINT                 = 0x0CC
    CURRENT_CONTROLLER_I_Q_INTEGRATOR               = 0x0D0
    CURRENT_CONTROLLER_I_D_INTEGRATOR               = 0x0D4
    POWERSTAGE_HTIM                                 = 0x0D8
    POWERSTAGE_HADC1                                = 0x0DC
    POWERSTAGE_HADC2                                = 0x0E0
    POWERSTAGE_ADC_READING_RAW                      = 0x0E4
    POWERSTAGE_ADC_READING_OFFSET                   = 0x0EC
    POWERSTAGE_UNDERVOLTAGE_THRESHOLD               = 0x0F4
    POWERSTAGE_OVERVOLTAGE_THRESHOLD                = 0x0F8
    POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA             = 0x0FC
    POWERSTAGE_BUS_VOLTAGE_MEASURED                 = 0x100
    MOTOR_POLE_PAIRS                                = 0x104
    MOTOR_TORQUE_CONSTANT                           = 0x108
    MOTOR_PHASE_ORDER                               = 0x10C
    MOTOR_MAX_CALIBRATION_CURRENT                   = 0x110
    ENCODER_HI2C                                    = 0x114
    ENCODER_I2C_BUFFER                              = 0x118
    ENCODER_I2C_UPDATE_COUNTER                      = 0x11C
    ENCODER_CPR                                     = 0x120
    ENCODER_POSITION_OFFSET                         = 0x124
    ENCODER_VELOCITY_FILTER_ALPHA                   = 0x128
    ENCODER_POSITION_RAW                            = 0x12C
    ENCODER_N_ROTATIONS                             = 0x130
    ENCODER_POSITION                                = 0x134
    ENCODER_VELOCITY                                = 0x138
    ENCODER_FLUX_OFFSET                             = 0x13C
    ENCODER_FLUX_OFFSET_TABLE                       = 0x140
    # end: 840   0x348


class DataFrame:
    def __init__(self, 
            device_id: int = 0, 
            func_id: Function = None, 
            size: int = 0, 
            data: bytes = b""
        ):
        self.device_id = device_id
        self.func_id = func_id
        self.size = size
        self.data = data

        assert self.size == len(self.data)


class Transport:
    """
    @param port: port of the transport
    """
    def __init__(self, port):
        self.port = port
        self.devices = []
        self._killed = threading.Event()
    
    """
    Receive data

    timeout == None: blocking forever
    timeout == 0: non-blocking (the actual delay is around 0.1s)
    timeout > 0: blocking for timeout seconds

    @param timeout: timeout in seconds
    """
    def receive(self, match_function: Function = None, match_device_id: int = None, timeout=None) -> DataFrame:
        raise NotImplementedError

    def transmit(self, frame: DataFrame):
        raise NotImplementedError


from .can import CANFrame

class MotorController:
    def __init__(self, transport: Transport, id: int = 1, name=""):
        self.transport: Transport = transport
        self.id: int = id
        self.name = name

        self.transport.devices.append(self)
        
        self._position_offset: float = 0

    @staticmethod
    def unpack(format_str, data) -> tuple:
        try:
            return struct.unpack(format_str, data)
        except struct.error as e:
            print("warning:", e, data)
            return []

    def ping(self, timeout=0.1) -> bool:
        self.transport.transmit(CANFrame(self.id, Function.RECEIVE_PDO_1, size=1, data=b"\xCA"))
        rx_frame = self.transport.receive(timeout=timeout, match_function=Function.TRANSMIT_PDO_1, match_device_id=self.id)
        if not rx_frame:
            return False
        rx_data = MotorController.unpack("<BBBBBBBB", rx_frame.data)[0]
        return rx_data == 0xCA

    def feed(self) -> None:
        self.transport.transmit(CANFrame(self.id, Function.HEARTBEAT))

    def set_mode(self, mode: Mode) -> None:
        self.transport.transmit(CANFrame(
            self.id,
            Function.NMT,
            size=2,
            data=struct.pack("<BB", mode, self.id)
        ))
        
    def load_settings_from_flash(self) -> None:
        self.transport.transmit(CANFrame(
            self.id,
            Function.FLASH, 
            size=1,
            data=struct.pack("<B", 2)
        ))
    
    def store_settings_to_flash(self) -> None:
        self.transport.transmit(CANFrame(
            self.id,
            Function.FLASH, 
            size=1, 
            data=struct.pack("<B", 1)
        ))
    
    def _read_parameter(self, param_id: Parameter, timeout=None) -> bytes:
        self.transport.transmit(CANFrame(
            self.id,
            Function.RECEIVE_SDO, 
            size=3,
            data=struct.pack("<BH", 0x02 << 5, param_id)
        ))
        rx_frame = self.transport.receive(match_function=Function.TRANSMIT_SDO, match_device_id=self.id, timeout=timeout)
        return rx_frame

    def _write_parameter(self, param_id: Parameter, tx_data: bytes) -> None:
        self.transport.transmit(CANFrame(
            self.id,
            Function.RECEIVE_SDO, 
            size=8,
            data=struct.pack("<BHB", 0x01 << 5, param_id, 0) + tx_data
        ))

    def _read_parameter_bytes(self, param_id: Parameter, timeout=None) -> bytes:
        rx_frame = self._read_parameter(param_id, timeout)
        if not rx_frame:
            return None
        rx_data = rx_frame.data[0:4]
        return rx_data

    def _read_parameter_f32(self, param_id: Parameter, timeout=None) -> float:
        rx_frame = self._read_parameter(param_id, timeout)
        if not rx_frame:
            return None
        rx_data = MotorController.unpack("<f", rx_frame.data[0:4])[0]
        return rx_data

    def _read_parameter_i32(self, param_id: Parameter, timeout=None) -> int:
        rx_frame = self._read_parameter(param_id, timeout)
        if not rx_frame:
            return None
        rx_data = MotorController.unpack("<l", rx_frame.data[0:4])[0]
        return rx_data

    def _read_parameter_u32(self, param_id: Parameter, timeout=None) -> int:
        rx_frame = self._read_parameter(param_id, timeout)
        if not rx_frame:
            return None
        rx_data = MotorController.unpack("<L", rx_frame.data[0:4])[0]
        return rx_data

    def _write_parameter_bytes(self, param_id: Parameter, value: bytes):
        self._write_parameter(param_id, value)

    def _write_parameter_f32(self, param_id: Parameter, value: float):
        tx_data = struct.pack("<f", value)
        self._write_parameter(param_id, tx_data)

    def _write_parameter_i32(self, param_id: Parameter, value: int):
        assert type(value) == int, "value must be an integer"
        tx_data = struct.pack("<l", value)
        self._write_parameter(param_id, tx_data)

    def _write_parameter_u32(self, param_id: Parameter, value: int):
        assert type(value) == int, "value must be an integer"
        assert value >= 0, "value must be unsigned integer"
        tx_data = struct.pack("<L", value)
        self._write_parameter(param_id, tx_data)
    
    # Parameter fields

    def read_fast_frame_frequency(self) -> int:
        return self._read_parameter_u32(Parameter.FAST_FRAME_FREQUENCY)

    def write_fast_frame_frequency(self, value: int):
        self._write_parameter_u32(Parameter.FAST_FRAME_FREQUENCY, value)

    def read_gear_ratio(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_GEAR_RATIO)

    def write_gear_ratio(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_GEAR_RATIO, value)

    def read_position_kp(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_KP)

    def write_position_kp(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_KP, value)

    def read_position_kd(self) -> float:
        return self.read_velocity_kp()

    def write_position_kd(self, value: float):
        self.write_velocity_kp(value)

    def read_position_ki(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_KI)

    def write_position_ki(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_KI, value)

    def read_velocity_kp(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_KP)

    def write_velocity_kp(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_KP, value)

    def read_velocity_ki(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_KI)

    def write_velocity_ki(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_KI, value)

    def read_torque_limit(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_TORQUE_LIMIT)

    def write_torque_limit(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_TORQUE_LIMIT, value)
    
    def read_velocity_limit(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT)

    def write_velocity_limit(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT, value)

    def read_position_limit_lower(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER)

    def write_position_limit_lower(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER, value)

    def read_position_limit_upper(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER)

    def write_position_limit_upper(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER, value)

    def read_position_offset(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_OFFSET)

    def write_position_offset(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_OFFSET, value)

    def read_torque_target(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_TORQUE_TARGET)

    def write_torque_target(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_TORQUE_TARGET, value)
    
    def read_torque_measured(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_TORQUE_MEASURED)

    def read_velocity_target(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_TARGET)
    
    def write_velocity_target(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_TARGET, value)

    def read_velocity_measured(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_VELOCITY_MEASURED)

    def read_position_target(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_TARGET)
    
    def write_position_target(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_TARGET, value)
    
    def read_position_measured(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_POSITION_MEASURED)

    def read_torque_filter_alpha(self) -> float:
        return self._read_parameter_f32(Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA)

    def write_torque_filter_alpha(self, value: float):
        self._write_parameter_f32(Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA, value)


    def read_current_limit(self) -> float:
        return self._read_parameter_f32(Parameter.CURRENT_CONTROLLER_I_LIMIT)

    def write_current_limit(self, value: float):
        self._write_parameter_f32(Parameter.CURRENT_CONTROLLER_I_LIMIT, value)
    
    def read_current_kp(self) -> float:
        return self._read_parameter_f32(Parameter.CURRENT_CONTROLLER_I_KP)
    
    def write_current_kp(self, value: float):
        self._write_parameter_f32(Parameter.CURRENT_CONTROLLER_I_KP, value)

    def read_current_ki(self) -> float:
        return self._read_parameter_f32(Parameter.CURRENT_CONTROLLER_I_KI)
    
    def write_current_ki(self, value: float):
        self._write_parameter_f32(Parameter.CURRENT_CONTROLLER_I_KI, value)
    
    def read_bus_voltage_filter_alpha(self) -> float:
        return self._read_parameter_f32(Parameter.POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA)

    def write_bus_voltage_filter_alpha(self, value: float):
        self._write_parameter_f32(Parameter.POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA, value)

    def read_motor_pole_pairs(self) -> int:
        return self._read_parameter_u32(Parameter.MOTOR_POLE_PAIRS)
    
    def write_motor_pole_pairs(self, value: int):
        return self._write_parameter_u32(Parameter.MOTOR_POLE_PAIRS, value)
    
    def read_motor_torque_constant(self) -> float:
        return self._read_parameter_f32(Parameter.MOTOR_TORQUE_CONSTANT)
    
    def write_motor_torque_constant(self, value: float):
        return self._write_parameter_f32(Parameter.MOTOR_TORQUE_CONSTANT, value)

    def read_motor_phase_order(self) -> int:
        return self._read_parameter_i32(Parameter.MOTOR_PHASE_ORDER)
    
    def write_motor_phase_order(self, value: int):
        return self._write_parameter_i32(Parameter.MOTOR_PHASE_ORDER, value)
    
    def read_motor_calibration_current(self) -> float:
        return self._read_parameter_f32(Parameter.MOTOR_MAX_CALIBRATION_CURRENT)
    
    def write_motor_calibration_current(self, value: float):
        return self._write_parameter_f32(Parameter.MOTOR_MAX_CALIBRATION_CURRENT, value)


    def read_encoder_cpr(self) -> int:
        return self._read_parameter_u32(Parameter.ENCODER_CPR)
    
    def write_encoder_cpr(self, value: int):
        return self._write_parameter_u32(Parameter.ENCODER_CPR, value)
    
    def read_encoder_position_offset(self) -> float:
        return self._read_parameter_f32(Parameter.ENCODER_POSITION_OFFSET)
    
    def write_encoder_position_offset(self, value: float):
        return self._write_parameter_f32(Parameter.ENCODER_POSITION_OFFSET, value)
    
    def read_encoder_velocity_filter_alpha(self) -> float:
        return self._read_parameter_f32(Parameter.ENCODER_VELOCITY_FILTER_ALPHA)
    
    def write_encoder_velocity_filter_alpha(self, value: float):
        return self._write_parameter_f32(Parameter.ENCODER_VELOCITY_FILTER_ALPHA, value)
    
    def read_encoder_flux_offset(self) -> float:
        return self._read_parameter_f32(Parameter.ENCODER_FLUX_OFFSET)
    
    def write_encoder_flux_offset(self, value: float):
        return self._write_parameter_f32(Parameter.ENCODER_FLUX_OFFSET, value)


    # Quick helper functions

    def set_current_bandwidth(self, bandwidth_hz: float, phase_resistance: float, phase_inductance: float):
        kp = bandwidth_hz * 2.0 * math.pi * phase_inductance
        ki = phase_resistance / phase_inductance
        print(f"Calculated current kp: {kp:.4f}, ki: {ki:.4f}")
        self.write_current_kp(kp)
        self.write_current_ki(ki)

    def set_torque_bandwidth(self, bandwidth_hz: float, position_loop_rate: float = 2000):
        alpha = min(max(1. - math.exp(-2. * math.pi * (bandwidth_hz / position_loop_rate)), 0.), 1.)
        print(f"Calculated torque filter alpha: {alpha:.4f}")
        self.write_torque_filter_alpha(alpha)

    def set_bus_voltage_bandwidth(self, bandwidth_hz: float, bus_voltage_update_rate: float = 10000):
        alpha = min(max(1. - math.exp(-2. * math.pi * (bandwidth_hz / bus_voltage_update_rate)), 0.), 1.)
        print(f"Calculated bus voltage filter alpha: {alpha:.4f}")
        self.write_bus_voltage_filter_alpha(alpha)
    
    def set_encoder_velocity_bandwidth(self, bandwidth_hz: float, encoder_update_rate: float = 10000):
        alpha = min(max(1. - math.exp(-2. * math.pi * (bandwidth_hz / encoder_update_rate)), 0.), 1.)
        print(f"Calculated encoder velocity filter alpha: {alpha:.4f}")
        self.write_encoder_velocity_filter_alpha(alpha)

    def write_pdo_2(self, position_target: float, velocity_target: float):
        self.transport.transmit(CANFrame(
            self.id,
            Function.RECEIVE_PDO_2,
            size=8,
            data=struct.pack("<ff", position_target, velocity_target)
        ))
        rx_frame = self.transport.receive(match_function=Function.TRANSMIT_PDO_2, match_device_id=self.id)

        if rx_frame:
            measured_position, measured_velocity = struct.unpack("<ff", rx_frame.data[0:8])
            return measured_position, measured_velocity
        else:
            return None, None
