import os
import time
import json
import random
import threading
import cv2
import numpy as np
import yaml
import sys
import tty
import termios
import can
from datetime import datetime
import pyrealsense2 as rs
from motorcontroller import MotorController

# ==========================================
# 📋 任务描述池 (Task Description Pool)
# ==========================================
TASK_DESCRIPTIONS = [
    "Pick up the screwdriver and place it into the nearby express box.",
    "Grab the screwdriver from the table and move it to the side express package.",
    "Retrieve the screwdriver and drop it inside the adjacent delivery box.",
    "Lift the screwdriver and put it in the express container positioned next to the arm.",
    "Grasp the screwdriver and transfer it into the express parcel box nearby."
]

class DataCollector:
    def __init__(self):
        self.controller = MotorController()
        self.is_recording = False
        self.is_running = True
        self.current_episode_path = ""
        self.frames_data = []
        
        # 默认每次按键移动 1.0 度
        self.step_size = 1.0  
        
        # 目标位置缓存
        self.targets = {i: 0.0 for i in range(1, 8)}
        
        # 各关节物理安全角度限制 (Min, Max)
        self.joint_limits = {
            1: (5.0, 340.0),
            2: (10.0, 180.0),
            3: (-180.0, 0.0),
            4: (-230.0, -10.0),
            5: (10.0, 220.0),
            6: (10.0, 280.0),
            7: (-120.0, 0.0) 
        }
        
        # 多相机管线列表
        self.pipelines = []
        
        # 频率控制
        self.record_hz = 30
        self.interval = 1.0 / self.record_hz

    def getch(self):
        """Linux 原生非阻塞读取单个按键输入"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            if ch == '\x03': # 捕获 Ctrl+C
                raise KeyboardInterrupt
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def start_hardware(self):
        print("[1/4] 正在加载 motors.yaml 并接入 CAN 总线...")
        with open('motors.yaml', 'r') as f:
            motor_config = yaml.safe_load(f)
        for node in motor_config['nodes']:
            self.controller.add_motor(node['id'], reduction=node['reduction'])
            
        if self.controller.is_initialized():
            self.controller.start()
            
            # ------------------ 新增：驱动至固定初始姿态 ------------------
            print("[2/4] 正在将机械臂驱动至设定的初始姿态...")
            initial_poses = {
                1: 180.0,
                2: 80.0,
                3: -100.0,
                4: -120.0,
                5: 115.0,
                6: 140.0,
                7: -115.0
            }
            
            for i in range(1, 8):
                motor = self.controller.motors.get(i)
                if motor:
                    motor.set_position(initial_poses[i])
                    print(f"      -> 指令下发: 关节 [{i}] 目标 {initial_poses[i]}°")
                    
            print("      等待机械臂到达初始位置 (3秒)...")
            time.sleep(3.0)

            # --------------------------------------------------------------
            print("[3/4] 正在重新读取底层真实电机位置，作为键盘控制的基准...")
            for i in range(1, 8):
                motor = self.controller.motors.get(i)
                if motor:
                    motor.reference_status()
                    
            time.sleep(0.5) 
            
            for i in range(1, 8):
                motor = self.controller.motors.get(i)
                if motor:
                    self.targets[i] = motor.position
                    print(f"      -> 关节 [{i}] 真实角度已同步: {self.targets[i]:.1f}°")
        else:
            print("错误: CAN 总线挂载失败。")
            return
        
        print("[4/4] 正在开启多路 D415 相机 (424x240 @ 30FPS)...")
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                print("⚠️ 警告：未检测到任何 RealSense 相机！")
            
            for i, dev in enumerate(devices):
                if i >= 3: break 
                sn = dev.get_info(rs.camera_info.serial_number)
                pipe = rs.pipeline(ctx)
                config = rs.config()
                config.enable_device(sn)
                
                config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
                pipe.start(config)
                self.pipelines.append(pipe)
                print(f"      -> 已成功绑定并启动相机 [{i}] (SN: {sn})")
        except Exception as e:
            print(f"相机启动异常: {e}")
        
        if not os.path.exists('datasets'):
            os.makedirs('datasets')

    def park_robot(self):
        """安全收臂：逆序逐一返回全局待机姿态"""
        homing_params = {
            1: 180.0, 2: 80.0, 3: -100.0, 
            4: -120.0, 5: 115.0, 6: 140.0, 7: -115.0
        }
        print("\n\n[系统] 正在执行安全收臂：从末端向基座逆序归位...")
        
        for m_id in sorted(homing_params.keys(), reverse=True):
            if m_id in self.controller.motors:
                pos = homing_params[m_id]
                self.targets[m_id] = pos  
                motor = self.controller.motors[m_id]
                motor.set_position(pos)
                
                reach_timeout = 10.0
                start_t = time.time()
                reached = False
                
                while True:
                    if time.time() - start_t > reach_timeout:
                        break
                        
                    curr_pos = motor.position
                    diff = abs(curr_pos - pos)
                    sys.stdout.write(f"\r      -> 正在收回 关节 [{m_id}] : 当前 {curr_pos:6.1f}° / 目标 {pos:6.1f}° (偏差: {diff:5.1f}°)   ")
                    sys.stdout.flush()
                    if diff < 1.5:
                        reached = True
                        break
                    time.sleep(0.05)
                
                print() 
                if reached:
                    print(f"      ✅ 关节 [{m_id}] 已就位。")
                    time.sleep(0.2) 
                else:
                    print(f"      ⚠️ 警告: 关节 [{m_id}] 移动超时！")
        
        print("[系统] 逆序归位全部完成！随时可进行下一步操作。\n")

    def auto_operate_gripper(self, action):
        """极速专属控制逻辑"""
        motor = self.controller.motors[7]
        MAX_OPEN = -115.0  # 修改：张开到-115度
        MAX_CLOSE = 0.0    # 修改：闭合到0度
        TORQUE_THRESHOLD = 0.03
        
        if action == 'open':
            sys.stdout.write("\n⚡ 夹爪极速张开 🖐️... ")
            sys.stdout.flush()
            self.targets[7] = MAX_OPEN
            motor.set_position(self.targets[7])
            print("[已瞬间弹开]")
            
        elif action == 'close':
            sys.stdout.write("\n⚡ 夹爪极速闭合 ✊... ")
            sys.stdout.flush()
            start_pos = motor.position
            motor.set_position(MAX_CLOSE)
            time.sleep(0.15)
            
            while True:
                tx_id = motor.build_can_id(dir_bit=0, cmd_id=0x0F) 
                with motor.lock:
                    motor.bus.send(can.Message(arbitration_id=tx_id, data=[2], is_extended_id=False))
                    motor.bus.send(can.Message(arbitration_id=tx_id, data=[0], is_extended_id=False))
                
                time.sleep(0.01) 
                
                curr_pos = motor.position
                torque = abs(getattr(motor, 'motor_torque', 0.0))
                
                if curr_pos >= MAX_CLOSE - 2.0:
                    self.targets[7] = MAX_CLOSE
                    motor.set_position(self.targets[7])
                    print(f"[空抓到底，未碰到物体] 最终位置: {curr_pos:.1f}°")
                    break
                    
                if torque > TORQUE_THRESHOLD and abs(curr_pos - start_pos) > 2.0:
                    self.targets[7] = curr_pos
                    motor.set_position(self.targets[7])
                    time.sleep(0.01)
                    motor.set_position(self.targets[7])
                    print(f"[🔒 砰！咬紧物体！急停死锁 (受力 Torque: {torque:.2f}, 夹取位置: {curr_pos:.1f}°)]")
                    break
                time.sleep(0.01)

    def keyboard_loop(self):
        """监听控制指令 (整合限位钳制与步长系统)"""
        mapping = {
            'q': (1, 1), 'a': (1, -1),
            'w': (2, 1), 's': (2, -1),
            'e': (3, 1), 'd': (3, -1),
            'r': (4, 1), 'f': (4, -1),
            't': (5, 1), 'g': (5, -1),
            'y': (6, 1), 'h': (6, -1),
            'u': (7, 1), 'j': (7, -1),
        }
        
        print("\n" + "="*60)
        print(" 🎮 键盘控制映射已激活 (按下立即生效):")
        print(" [Q/A] -> 关节1    [W/S] -> 关节2    [E/D] -> 关节3")
        print(" [R/F] -> 关节4    [T/G] -> 关节5    [Y/H] -> 关节6")
        print(" [U/J] -> 关节7")
        print("\n [=] -> 增大单次步长    [-] -> 减小单次步长")
        print(" 位置控制: [P] 逆序一键返回全局待机姿态 (Home)")
        print(" 录制控制: [C] 开始录制 | [V] 停止并保存 | [B] 回放上次序列")
        print(" [ESC] 或 [Ctrl+C] 退出当前脚本 (维持原状态抱死)")
        print("="*60 + "\n")

        try:
            while self.is_running:
                ch = self.getch().lower()
                
                if ch == '\x1b': # ESC 键退出
                    self.is_running = False
                    
                elif ch == '=' or ch == '+':
                    self.step_size += 0.5
                    sys.stdout.write(f"\r[步长调整] 当前按键步长放大为: {self.step_size:>4.1f}°" + " "*30)
                    sys.stdout.flush()
                    
                elif ch == '-':
                    self.step_size = max(0.1, self.step_size - 0.5)
                    sys.stdout.write(f"\r[步长调整] 当前按键步长缩小为: {self.step_size:>4.1f}°" + " "*30)
                    sys.stdout.flush()
                    
                elif ch in mapping:
                    m_id, direction = mapping[ch]
                    if m_id in self.controller.motors:
                        if m_id == 7 and direction == 1:
                            self.auto_operate_gripper('open')
                        elif m_id == 7 and direction == -1:
                            self.auto_operate_gripper('close')
                        else:
                            # 理论计算新的目标位置
                            theoretical_target = self.targets[m_id] + direction * self.step_size
                            
                            # 获取该关节的限位范围
                            min_limit, max_limit = self.joint_limits.get(m_id, (-360.0, 360.0))
                            
                            # 钳制逻辑与状态提示
                            limit_warning = ""
                            if theoretical_target > max_limit:
                                new_target = max_limit
                                limit_warning = f" ⚠️ 达正向极限({max_limit}°)"
                            elif theoretical_target < min_limit:
                                new_target = min_limit
                                limit_warning = f" ⚠️ 达负向极限({min_limit}°)"
                            else:
                                new_target = theoretical_target
    
                            # 覆盖旧目标值并下发
                            self.targets[m_id] = new_target
                            self.controller.motors[m_id].set_position(new_target)
                            
                            # 终端清行并打印实时状态
                            sys.stdout.write(f"\r[键盘操控] 关节 {m_id} 目标 -> {new_target:>6.1f}° | 步长: {self.step_size:>4.1f}°{limit_warning}" + " "*10)
                            sys.stdout.flush()
                        
                elif ch == 'p':
                    self.park_robot()
                elif ch == 'c':
                    if not self.is_recording: self.start_recording()
                elif ch == 'v':
                    if self.is_recording: self.stop_recording()
                elif ch == 'b':
                    self.replay_last_episode()
                    
        except KeyboardInterrupt:
            self.is_running = False

    def start_recording(self):
        base_dir = "datasets"
        os.makedirs(base_dir, exist_ok=True)
        
        existing_episodes = []
        for d in os.listdir(base_dir):
            if d.startswith("episode_") and os.path.isdir(os.path.join(base_dir, d)):
                try:
                    num = int(d.split("_")[1])
                    existing_episodes.append(num)
                except ValueError:
                    pass
                    
        next_ep_num = max(existing_episodes) + 1 if existing_episodes else 1
        self.current_episode_path = os.path.join(base_dir, f"episode_{next_ep_num}")
        
        img_base_path = os.path.join(self.current_episode_path, "images")
        os.makedirs(img_base_path, exist_ok=True)
        for i in range(len(self.pipelines)):
            os.makedirs(os.path.join(img_base_path, f"cam_{i}"), exist_ok=True)
        
        self.frames_data = []
        self.is_recording = True
        
        instruction = random.choice(TASK_DESCRIPTIONS)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        metadata = {
            "episode_id": next_ep_num,
            "task": "Pick up screwdriver",
            "instruction": instruction,
            "resolution": "424x240",
            "fps_target": self.record_hz,
            "cameras_count": len(self.pipelines),
            "timestamp": timestamp
        }
        with open(os.path.join(self.current_episode_path, "metadata.json"), 'w') as f:
            json.dump(metadata, f, indent=4)
            
        print(f"\n🔴 正在多视角同步录制! 序列: episode_{next_ep_num} | 描述: {instruction}")
        threading.Thread(target=self.record_loop).start()

    def record_loop(self):
        frame_idx = 0
        while self.is_recording:
            start_time = time.time()
            
            imgs = {}
            for cam_idx, pipe in enumerate(self.pipelines):
                try:
                    frames = pipe.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        imgs[cam_idx] = np.asanyarray(color_frame.get_data())
                    else:
                        imgs[cam_idx] = np.zeros((240, 424, 3), dtype=np.uint8)
                except Exception:
                    imgs[cam_idx] = np.zeros((240, 424, 3), dtype=np.uint8)
            
            positions = {}
            for i in range(1, 8):
                if i in self.controller.motors:
                    positions[i] = self.controller.motors[i].position
            
            saved_img_paths = {}
            for cam_idx, img in imgs.items():
                img_name = f"frame_{frame_idx:05d}.jpg"
                full_save_path = os.path.join(self.current_episode_path, "images", f"cam_{cam_idx}", img_name)
                cv2.imwrite(full_save_path, img)
                saved_img_paths[f"cam_{cam_idx}"] = f"cam_{cam_idx}/{img_name}"
            
            self.frames_data.append({
                "frame_idx": frame_idx,
                "images": saved_img_paths,
                "positions": positions,
                "timestamp": time.time()
            })
            
            frame_idx += 1
            
            elapsed = time.time() - start_time
            sleep_time = max(0, self.interval - elapsed)
            time.sleep(sleep_time)

    def stop_recording(self):
        self.is_recording = False
        with open(os.path.join(self.current_episode_path, "data.json"), 'w') as f:
            json.dump(self.frames_data, f, indent=4)
        print(f"\n✅ 录制停止。共保存 {len(self.frames_data)} 步多模态数据至 {self.current_episode_path}")

    def replay_last_episode(self):
        if not self.current_episode_path or self.is_recording:
            print("\n❌ 没有可回放的序列或正在录制中")
            return
        
        print(f"\n🎬 开始回放采集的轨迹 ({self.current_episode_path})...")
        with open(os.path.join(self.current_episode_path, "data.json"), 'r') as f:
            data = json.load(f)
            
        for step in data:
            start_t = time.time()
            for m_id_str, pos in step["positions"].items():
                m_id = int(m_id_str)
                if m_id in self.controller.motors:
                    self.controller.motors[m_id].set_position(pos)
            
            elapsed = time.time() - start_t
            time.sleep(max(0, self.interval - elapsed))
            
        print("\n✨ 回放结束。")

    def cleanup(self):
        print("\n\n正在安全退出脚本 (保持机械臂当前姿态与使能状态)...")
        for pipe in self.pipelines:
            try:
                pipe.stop()
            except Exception:
                pass
        self.controller.stop()
        print("✅ 已退出。机械臂已平滑留在当前位置。")

if __name__ == "__main__":
    collector = DataCollector()
    try:
        collector.start_hardware()
        collector.keyboard_loop()
    finally:
        collector.cleanup()