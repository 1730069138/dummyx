import can
from typing import Optional, Dict, List
from motor import Motor, CMD_ID_GET_SAVED_POSITION, CMD_ID_STATUSWORD_REPORT, CMD_GET_CONFIG, CMD_ID_GET_STATUS, CMD_ID_GET_VALUE1, CAN_CMD_SAVE_ALL_CONFIG, CAN_CMD_CALIB_START, CAN_CMD_CALIB_REPORT
import threading

class MotorController:
    def __init__(self, interface: str = 'socketcan', channel: str = 'can0'):
        try:
            self.bus = can.Bus(interface=interface, channel=channel, receive_own_messages=True, fd=True)
        except Exception as e:
            print(f"Failed to initialize CAN bus: {e}")
            self.bus = None
        self.motors: Dict[int, Motor] = {}
        self.message_dict = {}
        self.message_lock = threading.Lock()
        self.running = False
        self.notifier = None
    
    def add_motor(self, node_id: int, reduction: float) -> Motor:
        if node_id in self.motors:
            raise ValueError(f"Motor with node ID {node_id} already exists")
        if not self.bus:
            raise RuntimeError("CAN bus not initialized")
        motor = Motor(self.bus, node_id, reduction)
        self.motors[node_id] = motor
        return motor
    
    def on_message_received(self, msg):
        with self.message_lock:
            self.message_dict[msg.arbitration_id] = msg
            dir_bit = (msg.arbitration_id >> 10) & 0x1
            node_id = (msg.arbitration_id >> 5) & 0x1F
            cmd_id = msg.arbitration_id & 0x1F
            
            if dir_bit == 1 and node_id in self.motors and cmd_id == CMD_ID_STATUSWORD_REPORT:
                # print(f"Motor {node_id}: Received STATUSWORD_REPORT message")
                self.motors[node_id].update_status(msg)
            if dir_bit == 1 and node_id in self.motors and cmd_id == CMD_ID_GET_STATUS:
                # print(f"Motor {node_id}: Received STATUSWORD_REPORT message")
                self.motors[node_id].update_status(msg)
            if dir_bit == 1 and node_id in self.motors and cmd_id == CMD_ID_GET_VALUE1:
                # print(f"Motor {node_id}: Received GET_CONFIG message")
                self.motors[node_id].update_status_all(msg)
            if dir_bit == 1 and node_id in self.motors and cmd_id == CMD_GET_CONFIG:
                # print(f"Motor {node_id}: Received GET_CONFIG message")
                self.motors[node_id].update_configs(msg)
            if dir_bit == 1 and node_id in self.motors and cmd_id == CAN_CMD_SAVE_ALL_CONFIG:
                print(f"Motor {node_id}: Received SAVE_ALL_CONFIG message")
            if dir_bit == 1 and node_id in self.motors and cmd_id == CAN_CMD_CALIB_START:
                print(f"Motor {node_id}: Started CALIB_START command")
                self.motors[node_id].update_calibrate_status(msg)
            if dir_bit == 1 and node_id in self.motors and cmd_id == CAN_CMD_CALIB_REPORT:
                self.motors[node_id].update_calibrate_report(msg)
            if dir_bit == 1 and node_id in self.motors and cmd_id == CMD_ID_GET_SAVED_POSITION:
                self.motors[node_id].update_saved_position(msg)
    
    def start(self):
        if not self.running and self.bus:
            self.notifier = can.Notifier(self.bus, [self.on_message_received])
            self.running = True
    
    def stop(self):
        if self.running and self.notifier:
            self.notifier.stop()
            self.running = False
    
    def get_motor_status(self, node_id: int) -> Optional[dict]:
        if node_id in self.motors:
            return self.motors[node_id].get_status_dict()
        return None
    
    def get_all_motor_status(self) -> List[dict]:
        return [motor.get_status_dict() for motor in self.motors.values()]
    
    def is_initialized(self) -> bool:
        return self.bus is not None
