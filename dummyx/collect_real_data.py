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
        self.step_size = 5.0  # 键盘控制步长 (度)
        
        # 目标位置缓存 (同步当前真实位置)
        self.targets = {i: 0.0 for i in range(1, 8)}
        
        # 多相机管线列表
        self.pipelines = []
        
        # 频率控制
        self.record_hz = 30
        self.interval = 1.0 / self.record_hz

    def getch(self):
        """Linux 原生读取单个按键"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            if ch == '\x03': raise KeyboardInterrupt
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def start_hardware(self):
        print("[1/3] 正在启动 CAN 控制器...")
        with open('motors.yaml', 'r') as f:
            motor_config = yaml.safe_load(f)
        for node in motor_config['nodes']:
            self.controller.add_motor(node['id'], reduction=node['reduction'])
        if self.controller.is_initialized():
            self.controller.start()
            # 同步初始位置
            time.sleep(1.0)
            for i in range(1, 8):
                if i in self.controller.motors:
                    self.targets[i] = self.controller.motors[i].position
        
        print("[2/3] 正在开启多路 D415 相机 (424x240 @ 30FPS)...")
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                print("⚠️ 警告：未检测到任何 RealSense 相机！")
            
            for i, dev in enumerate(devices):
                if i >= 3: break  # 最多支持 3 台相机
                sn = dev.get_info(rs.camera_info.serial_number)
                pipe = rs.pipeline(ctx)
                config = rs.config()
                config.enable_device(sn)
                
                # 统一为模型训练所需要的 RGB 彩色流，下调至 424x240 30FPS
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
        HOME_POSITIONS = {
            1: 180.0,
            2: 80.0,
            3: -86.0,
            4: -130.0,
            5: 150.0,
            6: 130.0,
            7: 50.0
        }
        print("\n\n[系统] 正在执行安全收臂：从末端向基座逆序归位...")
        
        for m_id in sorted(HOME_POSITIONS.keys(), reverse=True):
            if m_id in self.controller.motors:
                pos = HOME_POSITIONS[m_id]
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

    def keyboard_loop(self):
        """监听控制指令"""
        mapping = {
            'q': (1, 1), 'a': (1, -1), 'w': (2, 1), 's': (2, -1),
            'e': (3, 1), 'd': (3, -1), 'r': (4, 1), 'f': (4, -1),
            't': (5, 1), 'g': (5, -1), 'y': (6, 1), 'h': (6, -1),
            'u': (7, 1), 'j': (7, -1),
        }
        
        print("\n" + "="*50)
        print("🎮 采集控制台已就绪:")
        print("关节控制: Q/A, W/S, E/D, R/F, T/G, Y/H, U/J")
        print("步长调节: Z (减小), X (增加)")
        print("位置控制: [P] 逆序一键返回全局待机姿态 (Home)")
        print("录制控制: [C] 开始录制 | [V] 停止并保存 | [B] 回放上次序列")
        print("退出脚本: Ctrl + C")
        print("="*50 + "\n")

        try:
            while self.is_running:
                ch = self.getch().lower()
                if ch in mapping:
                    m_id, direction = mapping[ch]
                    self.targets[m_id] += direction * self.step_size
                    self.controller.motors[m_id].set_position(self.targets[m_id])
                elif ch == 'x':
                    self.step_size = min(30.0, self.step_size + 1.0)
                    print(f"-> 当前步长: {self.step_size}°")
                elif ch == 'z':
                    self.step_size = max(1.0, self.step_size - 1.0)
                    print(f"-> 当前步长: {self.step_size}°")
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
        
        # 1. 扫描并计算下一个顺序的 Episode 序号
        existing_episodes = []
        for d in os.listdir(base_dir):
            if d.startswith("episode_") and os.path.isdir(os.path.join(base_dir, d)):
                try:
                    num = int(d.split("_")[1])
                    existing_episodes.append(num)
                except ValueError:
                    pass
                    
        next_ep_num = max(existing_episodes) + 1 if existing_episodes else 1
        
        # 2. 创建顺序命名的主文件夹
        self.current_episode_path = os.path.join(base_dir, f"episode_{next_ep_num}")
        
        # 动态为每台相机创建独立的子文件夹
        img_base_path = os.path.join(self.current_episode_path, "images")
        os.makedirs(img_base_path, exist_ok=True)
        for i in range(len(self.pipelines)):
            os.makedirs(os.path.join(img_base_path, f"cam_{i}"), exist_ok=True)
        
        self.frames_data = []
        self.is_recording = True
        
        # 随机抽取一个英文任务描述
        instruction = random.choice(TASK_DESCRIPTIONS)
        
        # 保存 Metadata (在内部保留真实绝对时间戳)
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
            
            # 1. 抓取多路最新图像
            imgs = {}
            for cam_idx, pipe in enumerate(self.pipelines):
                try:
                    frames = pipe.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        imgs[cam_idx] = np.asanyarray(color_frame.get_data())
                    else:
                        # 分辨率对应调整为 (240, 424, 3) 防奔溃
                        imgs[cam_idx] = np.zeros((240, 424, 3), dtype=np.uint8)
                except Exception:
                    imgs[cam_idx] = np.zeros((240, 424, 3), dtype=np.uint8)
            
            # 2. 抓取电机本体位置
            positions = {}
            for i in range(1, 8):
                if i in self.controller.motors:
                    positions[i] = self.controller.motors[i].position
            
            # 3. 保存多视角图像并生成标准 VLA 相对路径映射
            saved_img_paths = {}
            for cam_idx, img in imgs.items():
                img_name = f"frame_{frame_idx:05d}.jpg"
                
                # 图像真实存入对应相机的物理独立文件夹
                full_save_path = os.path.join(self.current_episode_path, "images", f"cam_{cam_idx}", img_name)
                cv2.imwrite(full_save_path, img)
                
                # 记录针对 Dataset 根目录的相对路径 (Unix 风格正斜杠)
                saved_img_paths[f"cam_{cam_idx}"] = f"cam_{cam_idx}/{img_name}"
            
            # 4. 记录同步的数据对 (融合了多机视角)
            self.frames_data.append({
                "frame_idx": frame_idx,
                "images": saved_img_paths,
                "positions": positions,
                "timestamp": time.time()
            })
            
            frame_idx += 1
            
            # 5. 频率时间补偿 (严格锁定 30Hz)
            elapsed = time.time() - start_time
            sleep_time = max(0, self.interval - elapsed)
            time.sleep(sleep_time)

    def stop_recording(self):
        self.is_recording = False
        with open(os.path.join(self.current_episode_path, "data.json"), 'w') as f:
            json.dump(self.frames_data, f, indent=4)
        print(f"✅ 录制停止。共保存 {len(self.frames_data)} 步多模态数据至 {self.current_episode_path}")

    def replay_last_episode(self):
        if not self.current_episode_path or self.is_recording:
            print("❌ 没有可回放的序列或正在录制中")
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
            
            # 严格按照录制时的 30Hz 频率复现
            elapsed = time.time() - start_t
            time.sleep(max(0, self.interval - elapsed))
            
        print("✨ 回放结束。")

    def cleanup(self):
        print("\n正在安全退出脚本 (保持机械臂当前姿态与使能状态)...")
        for pipe in self.pipelines:
            try:
                pipe.stop()
            except Exception:
                pass
        self.controller.stop()

if __name__ == "__main__":
    collector = DataCollector()
    try:
        collector.start_hardware()
        collector.keyboard_loop()
    finally:
        collector.cleanup()
