import can
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Dict, List

# CAN 命令定义
CMD_ID_MOTOR_DISABLE = 0x00
CMD_ID_MOTOR_ENABLE = 0x01
CMD_ID_SET_POSITION = 0x04
CAN_CMD_CALIB_START = 0x05
CAN_CMD_CALIB_REPORT  = 0x06
CMD_ID_SET_HOME = 0x0B
CMD_SET_CONFIG = 0x11
CMD_GET_CONFIG = 0x12
CMD_ID_GET_STATUS = 0x0D
CMD_ID_STATUSWORD_REPORT = 0x0E
CMD_ID_GET_VALUE1 = 0x0F
CMD_ID_GET_SAVED_POSITION = 24
CAN_CMD_SAVE_ALL_CONFIG = 0x13
CAN_CMD_ERROR_RESET = 12
CURRENT_LIMIT = 7
CONTROL_MODE_INDEX = 11
KP_GAIN = 12
KD_GAIN = 13
KI_GAIN = 14
PROFILE_SYNC = 17
PROFILE_VELOCITY = 23
PROFILE_ACCEL = 24
PROFILE_DECEL = 25
PROTECT_OVER_CURRENT = 28
PROFILE_NODE_ID = 30
TORQUE = 0
VELOCITY = 1
POSITION = 2
MOTOR_CURRENT = 3
BUS_VOLTAGE = 4
BUS_CURRENT = 5
MOTOR_POWER = 6

CONFIG_ITEMS = [CURRENT_LIMIT, CONTROL_MODE_INDEX, KP_GAIN, KD_GAIN, KI_GAIN, PROFILE_VELOCITY, PROFILE_ACCEL, PROFILE_DECEL, PROTECT_OVER_CURRENT]
STATUS_ITEMS = [TORQUE, VELOCITY, POSITION, MOTOR_CURRENT, BUS_VOLTAGE, BUS_CURRENT, MOTOR_POWER]

@dataclass
class MotorStatus:
    motor_enabled: bool = False
    target_reached: bool = False
    current_limit: bool = False
    self_test_error: bool = False
    encoder_error: bool = False
    over_voltage: bool = False
    under_voltage: bool = False
    over_current: bool = False

class MotorConfigs:
    def __init__(self):
        self.current_limit = {'current_limit': 0.0}
        self.control_mode = {'control_mode': 0}
        self.velocity = {'velocity': 0.0}
        self.acceleration = {'acceleration': 0.0}
        self.deceleration = {'deceleration': 0.0}
        self.protect_over_current = {'protect_over_current': 0.0}
        self.nodeid = {'node_id': 0}
        self.kp_gain = {'kp_gain': 60.0}
        self.kd_gain = {'kd_gain': 0.003}
        self.ki_gain = {'ki_gain': 0.0001}

class Motor:
    def __init__(self, bus: can.Bus, node_id: int, reduction: float):
        self.bus = bus
        self.send_timeout = None
        self.node_id = node_id
        self.reduction  = reduction
        self.status = MotorStatus()
        self.configs = MotorConfigs()
        self.last_message_time = 0
        # GUI independently checks status and position freshness using a monotonic clock.
        self.status_received_at = None
        self.position_received_at = None
        self.velocity_received_at = None
        self.current_received_at = None
        self.position = 0.0
        self.saved_position = 0.0
        self.motor_current = 0.0
        self.motor_torque = 0.0
        self.motor_velocity = 0.0
        self.bus_voltage = 0.0
        self.bus_current = 0.0
        self.motor_power = 0.0
        self.calibration_progress = 0
        self.lock = threading.Lock()
        
    def build_can_id(self, dir_bit: int, cmd_id: int) -> int:
        return (dir_bit << 10) | ((self.node_id & 0x1F) << 5) | (cmd_id & 0x1F)
    
    def enable(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_ID_MOTOR_ENABLE)
        msg = can.Message(
            arbitration_id=tx_id,
            data=[],
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)
        print(f"Motor {self.node_id}: Sent ENABLE command")


    def reference_status(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_ID_GET_STATUS)
        msg = can.Message(
            arbitration_id=tx_id,
            data=[],
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)


    def reference_saved_position(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_ID_GET_SAVED_POSITION)
        msg = can.Message(
            arbitration_id=tx_id,
            data=[],
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)


    def reference_value1(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_ID_GET_VALUE1)
        for item in STATUS_ITEMS:
            msg = can.Message(arbitration_id=tx_id, data=[item], is_extended_id=False)
            with self.lock:
                self.bus.send(msg, timeout=self.send_timeout)

    def reference_feedback(self, items, inter_request_delay=0.02):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_ID_GET_VALUE1)
        items = tuple(items)
        for index, item in enumerate(items):
            msg = can.Message(arbitration_id=tx_id, data=[item], is_extended_id=False)
            with self.lock:
                self.bus.send(msg, timeout=self.send_timeout)
            if index < len(items) - 1 and inter_request_delay:
                time.sleep(inter_request_delay)

    def reference_motion_feedback(self, inter_request_delay=0.02):
        """Request feedback required while detecting a homing collision."""
        self.reference_feedback((POSITION, VELOCITY, MOTOR_CURRENT), inter_request_delay)

    def reference_position_feedback(self):
        self.reference_feedback((POSITION,), 0)

    def reference_current_feedback(self):
        self.reference_feedback((MOTOR_CURRENT,), 0)

    def disable(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_ID_MOTOR_DISABLE)
        msg = can.Message(
            arbitration_id=tx_id,
            data=[],
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)
        print(f"Motor {self.node_id}: Sent DISABLE command")
    
    def degrees_to_turns(self, degrees: float) -> float:
        return degrees / 360.0 * self.reduction
    
    def turns_to_degrees(self, turns: float) -> float:
        return turns / self.reduction * 360.0

    def set_position(self, position: float):
        turns = self.degrees_to_turns(position)
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_ID_SET_POSITION)
        data = struct.pack("<f", turns)
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)
        print(f"Motor {self.node_id}: Sent SET_POSITION: {position} degrees")

    def set_float_config(self, index: int, value: float):
        """Write one floating-point drive setting through the serialized bus path."""
        if index not in {CURRENT_LIMIT, PROFILE_VELOCITY, PROFILE_ACCEL,
                         PROFILE_DECEL, PROTECT_OVER_CURRENT}:
            raise ValueError(f'unsupported float config index: {index}')
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_SET_CONFIG)
        msg = can.Message(arbitration_id=tx_id,
                          data=struct.pack("<If", index, float(value)),
                          is_extended_id=False)
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)

    def set_home(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_ID_SET_HOME)

        msg = can.Message(
            arbitration_id=tx_id,
            data=[],
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)

    def set_homing_velocities(self,  velocity: float, acceleration: float, deceleration: float):
        
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_SET_CONFIG)
        data = struct.pack("<If", PROFILE_VELOCITY, velocity)
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)

        data = struct.pack("<If", PROFILE_ACCEL, acceleration)
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)
        
        time.sleep(1)

        data = struct.pack("<If", PROFILE_DECEL, deceleration)
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)

    def set_homing_current(self,  homing_current: float):


        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_SET_CONFIG)
        data = struct.pack("<If", CURRENT_LIMIT, homing_current)
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)

        data = struct.pack("<If", PROTECT_OVER_CURRENT, homing_current)
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)

    def set_homing(self, homing_current: float, homing_position, homing_expected_position, timeout, tolerance):
        # 将电流设为较低值以保护电机
        self.get_config()
        time.sleep(2)
        original_current = self.configs.current_limit['current_limit']
        original_velocity = self.configs.velocity['velocity']
        original_acceleration = self.configs.acceleration['acceleration']
        original_deceleration = self.configs.deceleration['deceleration']

        self.set_homing_current(homing_current)
        self.set_homing_velocities(5.0, 10.0, 10.0)
        self.set_position(homing_position)
        start_time = time.time()


        while not self.status.over_current:
            # 检查是否超时
            if time.time() - start_time > timeout:
                print(f"Motor {self.node_id}: Homing timeout after {timeout} seconds!")
                return 'timeout on step 1'
                break
            
            time.sleep(0.1)  # 避免忙等待
            self.reference_status()  # 请求更新状态
        else:
            # 循环正常结束后执行此处（过流标志为真）
            print(f"Motor {self.node_id}: Over current detected.")
            # 将当前位置设为零点
            self.error_resets()
            self.disable()
            time.sleep(1)
            self.set_home()
            time.sleep(1)
            print(f"Motor {self.node_id}: original current set to {original_current}")
            self.set_homing_current(20)
            time.sleep(2)
            self.enable()
            time.sleep(1)
            # 最后将各关节移至预设角度并设置零点
            self.set_position(homing_expected_position)
            target_position = homing_expected_position

            start_time = time.time()

            while True:
                current_pos = self.position
                if abs(current_pos - target_position) <= tolerance:
                    self.error_resets()
                    time.sleep(1)
                    self.disable()
                    time.sleep(1)
                    self.set_home()
                    time.sleep(1)
                    self.enable()
                    time.sleep(1)
                    print(f"Motor {self.node_id}: acceleration set to {original_acceleration}, deceleration {original_deceleration}")
                    print(f"Motor {self.node_id}: Reached target position {current_pos:.2f} (within tolerance +-{tolerance})")
                    self.set_homing_velocities(7, 10, 10)
                    time.sleep(3)
                    return 'success'
                    break
                
                if time.time() - start_time > timeout:
                    print(f"Motor {self.node_id}: Position timeout after {timeout} seconds! Current position: {current_pos:.2f}")
                    return 'timeout on step 2'
                    break
                
                time.sleep(0.1)
                self.reference_value1() 

    def set_damping_mode(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_SET_CONFIG)
        data = struct.pack("<II", CONTROL_MODE_INDEX, int(4))
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)    

    def set_stop_damping_mode(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_SET_CONFIG)
        data = struct.pack("<II", CONTROL_MODE_INDEX, int(3))
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)    

    def set_speed_mode (self, value):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_SET_CONFIG)
        data = struct.pack("<If", PROFILE_VELOCITY, value[0])
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)    

        data = struct.pack("<If", PROFILE_ACCEL, value[1])
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)    

        data = struct.pack("<If", PROFILE_DECEL, value[2])
        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)    

    def set_config(self, value):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_SET_CONFIG)
        for index in CONFIG_ITEMS:

            if index == CURRENT_LIMIT:
                data = struct.pack("<If", index, value[0].value)
            if index == CONTROL_MODE_INDEX:
                print(f"Motor {self.node_id}: Sent SET_CONFIG index={index}, value={value[1].value}")
                data = struct.pack("<II", index, int(value[1].value))
            if index == PROFILE_VELOCITY:
                print(f"Motor {self.node_id}: Sent PROFILE_VELOCITY index={index}, value={value[2].value}")
                data = struct.pack("<If", index, value[2].value)
            if index == PROFILE_ACCEL:
                data = struct.pack("<If", index, value[3].value)
            if index == PROFILE_DECEL:
                data = struct.pack("<If", index, value[4].value)
            if index == PROTECT_OVER_CURRENT:
                data = struct.pack("<If", index, value[5].value)



            if index == KP_GAIN:
                data = struct.pack("<If", index, value[6].value)
            if index == KD_GAIN:
                data = struct.pack("<If", index, value[7].value)
            if index == KI_GAIN:
                data = struct.pack("<If", index, value[8].value)

            msg = can.Message(
                arbitration_id=tx_id,
                data=data,
                is_extended_id=False
            )
            with self.lock:
                self.bus.send(msg, timeout=self.send_timeout)    
    def save_configs(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CAN_CMD_SAVE_ALL_CONFIG)
        msg = can.Message(
            arbitration_id=tx_id,
            data=[],
            is_extended_id=False
        )
        with self.lock:
           self.bus.send(msg, timeout=self.send_timeout)

    def error_resets(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CAN_CMD_ERROR_RESET)
        msg = can.Message(
            arbitration_id=tx_id,
            data=[],
            is_extended_id=False
        )
        with self.lock:
           self.bus.send(msg, timeout=self.send_timeout)

    def get_config(self):
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CMD_GET_CONFIG)
        for index in CONFIG_ITEMS:
            data = [index, 0, 0, 0]
            msg = can.Message(
                arbitration_id=tx_id,
                data=data,
                is_extended_id=False
            )
            with self.lock:
                self.bus.send(msg, timeout=self.send_timeout)
            print(f"Motor {self.node_id}: Sent GET_CONFIG index={index}")

    def start_calibration(self):
        print(f"Motor {self.node_id}: Start calibration")
        tx_id = self.build_can_id(dir_bit=0, cmd_id=CAN_CMD_CALIB_START)
        msg = can.Message(
            arbitration_id=tx_id,
            data=[],
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg, timeout=self.send_timeout)

    def update_calibrate_status(self, msg: can.Message):
        if len(msg.data) >= 4:
            status = struct.unpack("<I", msg.data[0:4])[0]
            if status == 0:
                self.calibrate_status = "Calibrated"
            elif status == -1:
                self.calibrate_status = "Calibrating Invalid"
            elif status == -2:
                self.calibrate_status = "Calibrating Error"
            elif status == -3:
                self.calibrate_status = "Calibrating Calib invalid"
            else:
                self.calibrate_status = "Unknown"

    def update_saved_position(self, msg: can.Message):
         if len(msg.data) >= 4:
            value = struct.unpack("<f", msg.data[0:4])[0]
            self.saved_position = self.turns_to_degrees(value)

    def update_calibrate_report(self, msg: can.Message):
        if len(msg.data) >= 8:
            steps = struct.unpack("<I", msg.data[0:4])[0]
            value = struct.unpack("<f", msg.data[4:8])[0]
            self.calibration_progress  = steps
            if steps == 1:
                print(f"Phase Resistance: {value:.6f}")
            elif steps == 2:
                print(f"Phase Inductance: {value:.6f}")
            elif steps == 3:
                print(f"Pole Pairs: {value:.1f}")
            elif steps == 4:
                print(f"Encoder Direction: {value:.1f}")
            else:
                print(f"Encoder Offset: {value}")


    def update_configs(self, msg: can.Message):
        if len(msg.data) >= 8:
            index = struct.unpack("<I", msg.data[0:4])[0]
            with self.lock:
                if index == CURRENT_LIMIT:
                    value = struct.unpack("<f", msg.data[4:8])[0]
                    self.configs.current_limit['current_limit'] = value
                if index == CONTROL_MODE_INDEX:
                    value = struct.unpack("<I", msg.data[4:8])[0]
                    self.configs.control_mode['control_mode'] = value
                if index == PROFILE_VELOCITY:
                    value = struct.unpack("<f", msg.data[4:8])[0]
                    self.configs.velocity['velocity'] = value
                if index == PROFILE_ACCEL:
                    value = struct.unpack("<f", msg.data[4:8])[0]
                    self.configs.acceleration['acceleration'] = value
                if index == PROFILE_DECEL:
                    value = struct.unpack("<f", msg.data[4:8])[0]
                    self.configs.deceleration['deceleration'] = value
                if index == PROTECT_OVER_CURRENT:
                    value = struct.unpack("<f", msg.data[4:8])[0]
                    self.configs.protect_over_current['protect_over_current'] = value
                if index == PROFILE_NODE_ID:
                    value = struct.unpack("<I", msg.data[4:8])[0]
                    self.configs.nodeid['node_id'] = value
                if index == KP_GAIN:
                    value = struct.unpack("<f", msg.data[4:8])[0]
                    self.configs.kp_gain['kp_gain'] = value
                if index == KD_GAIN:
                    value = struct.unpack("<f", msg.data[4:8])[0]
                    self.configs.kd_gain['kd_gain'] = value
                if index == KI_GAIN:
                    value = struct.unpack("<f", msg.data[4:8])[0]

                    self.configs.ki_gain['ki_gain'] = value


    def update_status(self, msg: can.Message):
        if len(msg.data) >= 8:
            status, error = struct.unpack("<II", msg.data[0:8])
            with self.lock:
                self.status.motor_enabled = bool(status & 0x01)
                self.status.target_reached = bool(status & 0x02)
                self.status.current_limit = bool(status & 0x04)
                self.status.self_test_error = bool(error & 0x01)
                self.status.encoder_error = bool(error & 0x02)
                self.status.over_voltage = bool(error & 0x10000)
                self.status.under_voltage = bool(error & 0x20000)
                self.status.over_current = bool(error & 0x40000)
                self.last_message_time = time.time()
                self.status_received_at = time.monotonic()

    def update_status_all(self, msg: can.Message):
        if len(msg.data) >= 8:
            l, h = struct.unpack("<fI", msg.data[0:8])
            with self.lock:
                match h:
                    case 0:
                        self.motor_torque = l
                    case 1:
                        self.motor_velocity = l
                        self.velocity_received_at = time.monotonic()
                    case 2:
                        self.position = self.turns_to_degrees(l)
                        self.position_received_at = time.monotonic()
                    case 3:
                        self.motor_current = l
                        self.current_received_at = time.monotonic()
                    case  4:
                        self.bus_voltage = l
                    case  5:
                        self.bus_current = l
                    case  6:
                        self.motor_power = l
                    case _:
                        print(f"Motor {self.node_id}: Unknown status type {h}")




    
    def get_status_dict(self) -> dict:
        with self.lock:
            return {
                'node_id': self.node_id,
                'enabled': self.status.motor_enabled,
                'target_reached': self.status.target_reached,
                'current_limit': self.status.current_limit,
                'errors': {
                    'self_test': self.status.self_test_error,
                    'encoder': self.status.encoder_error,
                    'over_voltage': self.status.over_voltage,
                    'under_voltage': self.status.under_voltage,
                    'over_current': self.status.over_current,
                },
                'last_update': time.time() - self.last_message_time,
                'status_age': (time.monotonic() - self.status_received_at
                               if self.status_received_at is not None else float('inf')),
                'position_age': (time.monotonic() - self.position_received_at
                                 if self.position_received_at is not None else float('inf')),
                'status_received_at': self.status_received_at,
                'position_received_at': self.position_received_at,
                'velocity_received_at': self.velocity_received_at,
                'current_received_at': self.current_received_at,
                'position': self.position,
                'saved_position': self.saved_position,
                'velocity': self.motor_velocity,
                'torque': self.motor_torque,
                'current': self.motor_current,
                'bus_voltage': self.bus_voltage,
                'bus_current': self.bus_current,
                'motor_power': self.motor_power,
                'calibration_progress': self.calibration_progress,
            }
