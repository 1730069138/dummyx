# cli.py
import threading
import time
import json
from datetime import datetime
import yaml
from motorcontroller import MotorController
import cmd

# 全局状态变量
is_recording = False
recorded_data = []
recording_thread = None
recording_file = "motor_positions.json"
calibration_progress = 0
calibration_in_progress = False
homing_event = threading.Event()

class MotorCLI(cmd.Cmd):
    intro = "欢迎使用电机控制系统CLI。输入 'help' 查看命令列表。"
    prompt = "motor> "
    
    def __init__(self):
        super().__init__()
        self.controller = MotorController()
        self.load_motor_config()
        self.start_controller()
    
    def load_motor_config(self):
        """从YAML文件加载电机配置"""
        try:
            with open('motors.yaml', 'r') as file:
                motor_config = yaml.safe_load(file)
            
            for node in motor_config['nodes']:
                node_id = node['id']
                reduction = node['reduction']
                try:
                    self.controller.add_motor(node_id, reduction=reduction)
                    print(f"已初始化电机 {node_id}, 减速比: {reduction}")
                except Exception as e:
                    print(f"初始化电机 {node_id} 错误: {e}")
        except Exception as e:
            print(f"加载配置文件错误: {e}")
    
    def start_controller(self):
        """启动控制器"""
        if self.controller.is_initialized():
            self.controller.start()
            print("CAN总线通知器已启动")
        else:
            print("警告: CAN总线未初始化")
    
    def do_enable(self, arg):
        """启用所有电机"""
        for motor in self.controller.motors.values():
            motor.enable()
        print("所有电机已启用")
    
    def do_disable(self, arg):
        """禁用所有电机"""
        for motor in self.controller.motors.values():
            motor.disable()
        print("所有电机已禁用")
    
    def do_reset(self, arg):
        """复位所有电机错误"""
        for motor in self.controller.motors.values():
            motor.error_resets()
        print("所有电机错误已复位")
    
    def do_homing(self, arg):
        """执行回零操作"""
        if homing_event.is_set():
            print("回零操作正在进行中!")
            return
        
        def _homing_task():
            homing_event.set()
            try:
                i = 0
                print("回零开始...")
                for motor in self.controller.motors.values():
                    i += 1
                    if i == 1:
                        result = motor.set_homing(3.0, -360, 178, 30, 0.5)
                        print(f"电机 {i} 回零结果: {result}")
                    elif i == 2:
                        result = motor.set_homing(6.0, -360, 81, 30, 0.5)
                        print(f"电机 {i} 回零结果: {result}")
                    elif i == 3:
                        result = motor.set_homing(6.0, 360, -95, 30, 0.5)
                        print(f"电机 {i} 回零结果: {result}")
                    elif i == 5:
                        result = motor.set_homing(3.0, -360, 115, 30, 0.5)
                        print(f"电机 {i} 回零结果: {result}")
                    else:
                        time.sleep(1)
                        motor.disable()
                        time.sleep(1)
                        motor.set_home()
                        time.sleep(1)
                        motor.enable()
                        time.sleep(1)
                        print(f"电机 {i} 已设置零点")
            finally:
                homing_event.clear()
                print("回零操作完成")
        
        threading.Thread(target=_homing_task, daemon=True).start()
    
    def do_move(self, arg):
        """移动所有电机到指定位置 (度)
        示例: move 90.5"""
        try:
            position = float(arg)
            for motor in self.controller.motors.values():
                motor.set_position(position)
            print(f"所有电机移动到位置: {position}度")
        except ValueError:
            print("错误: 请输入有效的数字位置")
    
    def do_move_neg(self, arg):
        """移动所有电机到负位置
        示例: move_neg 45"""
        try:
            position = float(arg)
            for motor in self.controller.motors.values():
                motor.set_position(position * -1)
            print(f"所有电机移动到负位置: {-position}度")
        except ValueError:
            print("错误: 请输入有效的数字位置")
    
    def do_move_home(self, arg):
        """移动所有电机到零点位置"""
        for motor in self.controller.motors.values():
            motor.set_position(0)
        print("所有电机移动到零点位置")
    
    def do_move_ready(self, arg):
        """移动到准备位置"""
        i = 0
        for motor in self.controller.motors.values():
            i += 1
            if i == 2:
                motor.set_position(-80)
            elif i == 3:
                motor.set_position(-95)
            elif i == 5:
                motor.set_position(0)
            else:
                motor.set_position(0)
        print("所有电机移动到准备位置")
    
    def do_move_range(self, arg):
        """移动到最大范围位置"""
        i = 0
        for motor in self.controller.motors.values():
            i += 1
            if i == 2:
                motor.set_position(90)
            elif i == 3:
                motor.set_position(90)
            else:
                motor.set_position(0)
        print("所有电机移动到最大范围位置")
    
    def do_loop(self, arg):
        """循环移动电机
        示例: loop 45 (将正负45度之间循环)"""
        if homing_event.is_set():
            print("循环操作正在进行中!")
            return
        
        try:
            position = float(arg)
        except ValueError:
            print("错误: 请输入有效的数字位置")
            return
        
        def _loop_task():
            homing_event.set()
            try:
                for _ in range(10):    
                    for motor in self.controller.motors.values():
                        motor.set_position(position * -1)
                    time.sleep(7)
                    for motor in self.controller.motors.values():
                        motor.set_position(position)
                    time.sleep(7)
                print("循环移动完成")
            finally:
                homing_event.clear()
        
        threading.Thread(target=_loop_task, daemon=True).start()
        print(f"开始循环移动, 位置: ±{position}度")
    
    def do_record_start(self, arg):
        """开始录制电机位置"""
        global is_recording, recorded_data, recording_thread
        
        if is_recording:
            print("录制已在进行中!")
            return
        
        is_recording = True
        recorded_data = []
        print("录制开始...")
        
        def record_positions():
            while is_recording:
                positions = []
                for motor_id, motor in self.controller.motors.items():
                    # 获取电机状态
                    status = motor.get_status_dict()
                    positions.append({
                        'node_id': motor_id,
                        'position': status['position']
                    })
                
                # 添加时间戳
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                recorded_data.append({
                    'timestamp': timestamp,
                    'positions': positions
                })
                
                time.sleep(0.5)  # 每0.5秒记录一次
        
        recording_thread = threading.Thread(target=record_positions, daemon=True)
        recording_thread.start()
    
    def do_record_stop(self, arg):
        """停止录制并保存数据"""
        global is_recording
        
        if not is_recording:
            print("没有正在进行的录制!")
            return
        
        is_recording = False
        print("录制停止")
        
        try:
            with open(recording_file, 'w') as f:
                json.dump(recorded_data, f, indent=4)
            print(f"数据已保存到 {recording_file}")
        except Exception as e:
            print(f"保存数据错误: {str(e)}")
    
    def do_replay(self, arg):
        """回放录制的电机位置"""
        global is_recording
        
        if is_recording:
            print("录制期间无法回放!")
            return
        
        try:
            with open(recording_file, 'r') as f:
                data = json.load(f)
        except Exception as e:
            print(f"加载录制文件错误: {str(e)}")
            return
        
        if not data:
            print("没有可回放的录制数据!")
            return
        
        print(f"开始回放 {len(data)} 个位置记录")
        
        def replay_task():
            for record in data:
                positions = record['positions']
                for pos in positions:
                    motor_id = pos['node_id']
                    position = pos['position']
                    if motor_id in self.controller.motors:
                        self.controller.motors[motor_id].set_position(position)
                time.sleep(0.5)  # 保持与记录时相同的时间间隔
        
        threading.Thread(target=replay_task, daemon=True).start()
    
    def do_calibrate(self, arg):
        """执行电机校准"""
        global calibration_progress, calibration_in_progress
        
        if calibration_in_progress:
            print("校准已在运行中!")
            return
        
        calibration_in_progress = True
        calibration_progress = 0
        
        print(f"开始校准电机...")
        
        def _calibration_task():
            global calibration_progress
            global calibration_in_progress
            try:
                i = 0
                for motor in self.controller.motors.values():                   
                    if motor.node_id == int(arg):
                        motor.start_calibration()
                        print(f"电机 {i} 开始校准...")
                        while motor.calibration_progress < 137:
                            time.sleep(0.1)
                            calibration_progress = motor.calibration_progress
                            print(f"校准进度: {calibration_progress}%")
            finally:
                calibration_in_progress = False
                print("校准完成")
        
        threading.Thread(target=_calibration_task, daemon=True).start()

    def do_save(self, arg):        
        """执行电机校准 arg is motor id"""
        i = 0
        print(f"电机 {i} configs save to MCU...")
        for motor in self.controller.motors.values():                   
            if motor.node_id == int(arg):
                motor.save_configs()        

    def do_status(self, arg):
        """显示所有电机状态"""
        status_data = []
        for motor in self.controller.motors.values():
            motor.reference_status()
            motor.reference_value1()
            
        for motor in self.controller.motors.values():
            status = motor.get_status_dict()
            errors = [k for k, v in status['errors'].items() if v]
            
            status_data.append({
                '节点ID': status['node_id'],
                '使能': '是' if status['enabled'] else '否',
                '目标到达': '是' if status['target_reached'] else '否',
                '错误': ', '.join(errors) if errors else '无',
                '位置': f"{status['position']:.3f}",
                '速度': f"{status['velocity']:.3f}",
                '扭矩': f"{status['torque']:.3f}",
                '电流': f"{status['current']:.3f}",
                '总线电压': f"{status['bus_voltage']:.3f}",
                '电机功率': f"{status['motor_power']:.3f}",
                '更新时间': f"{status['last_update']:.1f}秒前"
            })
        
        # 打印表格格式的状态
        if status_data:
            headers = status_data[0].keys()
            rows = [list(item.values()) for item in status_data]
            
            # 计算每列最大宽度
            col_widths = []
            for i, header in enumerate(headers):
                max_width = len(str(header))
                for row in rows:
                    if len(str(row[i])) > max_width:
                        max_width = len(str(row[i]))
                col_widths.append(max_width)
            
            # 打印表头
            header_row = " | ".join(f"{h:<{w}}" for h, w in zip(headers, col_widths))
            print(header_row)
            print("-" * len(header_row))
            
            # 打印数据行
            for row in rows:
                print(" | ".join(f"{str(item):<{w}}" for item, w in zip(row, col_widths)))
        else:
            print("没有电机状态数据")
    
    def do_exit(self, arg):
        """退出程序"""
        print("正在关闭...")
        self.controller.stop()
        return True
    
    def emptyline(self):
        """空行时不执行任何操作"""
        pass

if __name__ == '__main__':
    try:
        MotorCLI().cmdloop()
    except KeyboardInterrupt:
        print("\n程序已终止")
