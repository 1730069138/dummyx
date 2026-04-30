import os
os.environ["no_proxy"] = "100.64.142.55"
os.environ["NO_PROXY"] = "100.64.142.55"

import time
import cv2
import numpy as np
import yaml
import threading
import shutil
import json
from datetime import datetime
import pyrealsense2 as rs
from motorcontroller import MotorController
from openpi_client import websocket_client_policy

# ==========================================
# 🔑 1. 核心网络与硬件配置
# ==========================================
SERVER_IP = "100.64.142.55"  
SERVER_PORT = 8000
PROMPT = "Pick up the screwdriver and place it into the nearby express box."

SN_GLOBAL, SN_WRIST = "821312060126", "121622061691"

# ==========================================
# ⚙️ 2. ACT 时序融合与初始执行参数
# ==========================================
EXECUTION_HZ = 6.0   
EXEC_INTERVAL = 1.0 / EXECUTION_HZ
MAX_DEG_PER_STEP = 45.0 / EXECUTION_HZ  

STANDBY_POSITIONS = {
    1: 180.001, 2: 80.001, 3: -85.995, 
    4: -130.001, 5: 150.001, 6: 130.0, 7: 0.0
}

is_running = True

# 全局最新画面缓存与数据记录
latest_images = {"global": None, "wrist": None}
frames_data = []

# ==========================================
# 🧠 3. ACT 时序融合池
# ==========================================
class ACTBuffer:
    def __init__(self):
        self.buffer = {}  
        self.lock = threading.Lock()
        self.max_received_step = 0

    def add_chunk(self, start_step, chunk):
        with self.lock:
            for i, action in enumerate(chunk):
                abs_step = start_step + i
                if abs_step not in self.buffer:
                    self.buffer[abs_step] = []
                self.buffer[abs_step].append(action)
                self.max_received_step = max(self.max_received_step, abs_step)

    def get_fused_action(self, step_idx):
        with self.lock:
            if step_idx not in self.buffer:
                return None, 0
            
            preds = np.array(self.buffer[step_idx])
            fused_action = np.mean(preds, axis=0) 
            
            keys_to_delete = [k for k in self.buffer.keys() if k < step_idx]
            for k in keys_to_delete:
                del self.buffer[k]
                
            return fused_action, len(preds)

act_buffer = ACTBuffer()
current_global_step = 0  

# ==========================================
# 📷 4. 硬件初始化与线程解耦读取
# ==========================================
def init_camera_by_sn(sn, name):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(sn)
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 60)
    pipeline.start(config)
    return pipeline

def camera_worker(p_g, p_w):
    """专属洗帧线程：疯狂读取相机，保证 latest_images 永远是最新鲜的帧，且不阻塞主线程"""
    global is_running, latest_images
    while is_running:
        try:
            succ_g, f_g = p_g.try_wait_for_frames(timeout_ms=50)
            if succ_g and f_g.get_color_frame():
                img = np.asanyarray(f_g.get_color_frame().get_data())
                latest_images["global"] = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except: pass
        
        try:
            succ_w, f_w = p_w.try_wait_for_frames(timeout_ms=50)
            if succ_w and f_w.get_color_frame():
                img = np.asanyarray(f_w.get_color_frame().get_data())
                latest_images["wrist"] = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except: pass

def recording_worker(controller):
    """专属录制线程：以标准的 30Hz 频率录制与 collect_data.py 完全一致的数据集"""
    global is_running, latest_images, frames_data
    
    temp_dir = "datasets/temp_episode"
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir, ignore_errors=True)
    os.makedirs(os.path.join(temp_dir, "images/cam_0"), exist_ok=True)
    os.makedirs(os.path.join(temp_dir, "images/cam_1"), exist_ok=True)
    
    metadata = {
        "episode_id": "temp",
        "task": "Pick up screwdriver",
        "instruction": PROMPT,
        "resolution": "424x240",
        "fps_target": 30,
        "cameras_count": 2,
        "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S")
    }
    with open(os.path.join(temp_dir, "metadata.json"), 'w') as f:
        json.dump(metadata, f, indent=4)
        
    frame_idx = 0
    interval = 1.0 / 30.0
    
    while is_running:
        start_t = time.time()
        img_g = latest_images["global"]
        img_w = latest_images["wrist"]
        
        if img_g is not None and img_w is not None:
            positions = {}
            for i in range(1, 8):
                if i in controller.motors:
                    positions[i] = controller.motors[i].position
                    
            name_g = f"frame_{frame_idx:05d}.jpg"
            cv2.imwrite(os.path.join(temp_dir, "images/cam_0", name_g), cv2.cvtColor(img_g, cv2.COLOR_RGB2BGR))
            
            name_w = f"frame_{frame_idx:05d}.jpg"
            cv2.imwrite(os.path.join(temp_dir, "images/cam_1", name_w), cv2.cvtColor(img_w, cv2.COLOR_RGB2BGR))
            
            frames_data.append({
                "frame_idx": frame_idx,
                "images": {"cam_0": f"cam_0/{name_g}", "cam_1": f"cam_1/{name_w}"},
                "positions": positions,
                "timestamp": time.time()
            })
            frame_idx += 1
            
        elapsed = time.time() - start_t
        time.sleep(max(0, interval - elapsed))

def init_motors():
    controller = MotorController(interface='socketcan', channel='can0')
    controller.start()
    try:
        with open('motors.yaml', 'r') as f:
            motor_config = yaml.safe_load(f)
        for node in motor_config['nodes']:
            controller.add_motor(node['id'], reduction=node['reduction'])
    except Exception as e:
        print(f"❌ 读取 motors.yaml 失败: {e}")
    return controller

def return_to_standby(controller):
    print("\n\n🔄 正在执行平滑软着陆归位...")
    start_positions = {}
    for i in range(1, 8):
        if i in controller.motors:
            start_positions[i] = controller.motors[i].position

    for i in range(1, 8):
        if i in controller.motors:
            controller.motors[i].error_resets()
    time.sleep(0.2)
    
    for i in range(1, 8):
        if i in controller.motors:
            controller.motors[i].enable()
            controller.motors[i].set_position(start_positions[i])
    time.sleep(0.5)

    interp_steps = 150
    for step in range(1, interp_steps + 1):
        for i in range(1, 8):
            if i in controller.motors:
                curr_target = start_positions[i] + (STANDBY_POSITIONS[i] - start_positions[i]) * (step / interp_steps)
                controller.motors[i].set_position(curr_target)
        time.sleep(0.02) 

    reach_timeout = 5.0
    start_t = time.time()
    while True:
        if time.time() - start_t > reach_timeout:
            break
        all_reached = True
        for i in range(1, 8):
            if i in controller.motors:
                if abs(controller.motors[i].position - STANDBY_POSITIONS[i]) > 1.5:
                    all_reached = False
                    break
        if all_reached:
            print("✅ 机械臂已完美回到初始待机姿态！")
            time.sleep(0.5) 
            break
        time.sleep(0.1)

# ==========================================
# 🧵 5. 双摄独立推理线程 (动态频率拉伸)
# ==========================================
def inference_thread(controller):
    global is_running, current_global_step, act_buffer, latest_images
    global EXEC_INTERVAL, MAX_DEG_PER_STEP  
    
    np.set_printoptions(precision=2, suppress=True, linewidth=120)
    policy = None  
    
    while is_running:
        if policy is None:
            try:
                policy = websocket_client_policy.WebsocketClientPolicy(host=SERVER_IP, port=SERVER_PORT)
                print("\n✅ 成功连接到大模型推理服务器！")
            except Exception as e:
                print(f"\r⏳ 正在等待/重连大模型服务器... ({e})", end="")
                time.sleep(1.0)
                continue

        img_g = latest_images["global"]
        img_w = latest_images["wrist"]
        
        if img_g is None or img_w is None:
            time.sleep(0.01)
            continue
            
        curr_qpos = np.array([controller.motors[i].position for i in range(1, 8)])
        obs = {"cam_global": img_g, "cam_wrist": img_w, "state": curr_qpos, "prompt": PROMPT}
        
        img_start_step = current_global_step
        
        try:
            t_start = time.time()
            res = policy.infer(obs)
            t_cost = time.time() - t_start
            
            raw_chunk = np.array(res["actions"]) 
            chunk_len = len(raw_chunk)
            
            if np.max(np.abs(raw_chunk)) < 6.3 and np.max(np.abs(curr_qpos)) > 20:
                raw_chunk = raw_chunk * (180.0 / np.pi)
                
            if t_cost > 0 and chunk_len > 0:
                target_hz = (chunk_len * 0.7) / t_cost
                target_hz = np.clip(target_hz, 20.0, 30.0) 
                
                EXEC_INTERVAL = 1.0 / target_hz
                MAX_DEG_PER_STEP = 45.0 / target_hz  
                
            act_buffer.add_chunk(img_start_step, raw_chunk)
            
        except Exception as e:
            if is_running: 
                print(f"\n❌ 推理通信异常: {e}")
                policy = None  
            time.sleep(0.5)

# ==========================================
# 🚀 6. 主程序
# ==========================================
def main():
    global is_running, current_global_step, frames_data
    global EXEC_INTERVAL, MAX_DEG_PER_STEP
    
    controller = init_motors()
    p_g = init_camera_by_sn(SN_GLOBAL, "全局")
    p_w = init_camera_by_sn(SN_WRIST, "手腕")
    
    time.sleep(2)
    print("\n✅ VLA 动作部署启动 (后台数据集 30Hz 同步录制中)！")
    
    cam_thread = threading.Thread(target=camera_worker, args=(p_g, p_w))
    cam_thread.daemon = True
    cam_thread.start()
    
    rec_thread = threading.Thread(target=recording_worker, args=(controller,))
    rec_thread.daemon = True
    rec_thread.start()
    
    brain_thread = threading.Thread(target=inference_thread, args=(controller,))
    brain_thread.daemon = True
    brain_thread.start()
    
    home_arr = np.array([STANDBY_POSITIONS[i] for i in range(1, 7)])
    home_counter = 0
    HOME_THRESHOLD = 12 
    has_left_home = False  
    
    try:
        while is_running:
            loop_start = time.time()
            fused_target, num_preds = act_buffer.get_fused_action(current_global_step)
            
            if fused_target is not None:
                curr_qpos = np.array([controller.motors[i].position for i in range(1, 8)])
                diff_from_home = np.max(np.abs(curr_qpos[:6] - home_arr))
                
                if not has_left_home and diff_from_home > 15.0:
                    has_left_home = True
                    print("\n🚀 机械臂已离开待机位，开始执行抓取任务！")
                
                if has_left_home and diff_from_home < 5.0:
                    home_counter += 1
                else:
                    home_counter = 0
                    
                if home_counter >= HOME_THRESHOLD:
                    print("\n\n🎉 机械臂已完成任务并主动返回 P 键待机位！判定任务结束，准备收车...")
                    break 
                
                delta = fused_target - curr_qpos
                delta[:6] = np.clip(delta[:6], -MAX_DEG_PER_STEP, MAX_DEG_PER_STEP)
                delta[6] = np.clip(delta[6], -MAX_DEG_PER_STEP * 3, MAX_DEG_PER_STEP * 3)
                safe_target = curr_qpos + delta
                
                if safe_target[6] > curr_qpos[6]:
                    motor_7 = controller.motors[7]
                    torque_7 = abs(getattr(motor_7, 'torque', getattr(motor_7, 'current', 0.0)))
                    
                    TORQUE_THRESHOLD = 0.03
                    if torque_7 > TORQUE_THRESHOLD:
                        safe_target[6] = curr_qpos[6]
                        gripper_state = f'受阻锁死🔒 (力矩:{torque_7:.3f})'
                    else:
                        gripper_state = f'正在闭合✊'
                else:
                    gripper_state = f'正在张开🖐️'
                
                for m_id in range(1, 8):
                    controller.motors[m_id].set_position(safe_target[m_id-1])
                
                current_hz = 1.0 / EXEC_INTERVAL
                print(f"\r[{current_hz:.1f}Hz执行] 步数 {current_global_step:4d} | 离原点偏差:{diff_from_home:.1f}° | 夹爪:{gripper_state}        ", end="")
                current_global_step += 1
            else:
                print("\r[初始化/极限卡顿] 大脑算力严重不足，正在等待...                   ", end="")
                
            elapsed = time.time() - loop_start
            time.sleep(max(0, EXEC_INTERVAL - elapsed))

    except KeyboardInterrupt:
        print("\n\n🛑 收到人工中断信号！")
    except Exception as e:
        print(f"\n\n❌ 主循环发生致命异常: {e}")
    finally:
        is_running = False 
        
        # 1. 安全降落与关闭总线
        return_to_standby(controller) 
        print("正在断开 CAN 总线...")
        controller.stop()
        time.sleep(0.5) 
        try: p_g.stop(); p_w.stop();
        except: pass

        # ========================================================
        # 🚨 2. 新增：实验数据统计与人工判卷系统
        # ========================================================
        metrics_file = "experiment_metrics.json"
        metrics = {"total_trials": 0, "success_count": 0, "collision_count": 0}
        
        if os.path.exists(metrics_file):
            try:
                with open(metrics_file, 'r') as f:
                    metrics = json.load(f)
            except Exception:
                pass
                
        metrics["total_trials"] += 1
        
        print("\n" + "="*50)
        print("📊 【实验结果人工判卷系统】")
        ans_success = input("1. 本次实验是否【成功完成】抓取与放置？(y/n): ").strip().lower()
        ans_collision = input("2. 本次实验是否发生【意外碰撞】？(y/n): ").strip().lower()
        
        if ans_success == 'y':
            metrics["success_count"] += 1
        if ans_collision == 'y':
            metrics["collision_count"] += 1
            
        success_rate = (metrics["success_count"] / metrics["total_trials"]) * 100
        collision_rate = (metrics["collision_count"] / metrics["total_trials"]) * 100
        
        with open(metrics_file, 'w') as f:
            json.dump(metrics, f, indent=4)
            
        print(f"\n📈 累计数据看板 -> 总启动次数: {metrics['total_trials']} | 成功率: {success_rate:.1f}% | 碰撞率: {collision_rate:.1f}%")

        # ========================================================
        # 3. 数据集保存抉择逻辑
        # ========================================================
        if len(frames_data) > 0:
            print("\n" + "="*50)
            choice = input(f"💾 本次推理共后台录制了 {len(frames_data)} 帧 (30Hz) 完整轨迹数据！\n是否将本次运行保存为新的数据集？(y/n): ")
            if choice.lower() == 'y':
                base_dir = "datasets"
                os.makedirs(base_dir, exist_ok=True)
                existing_episodes = []
                for d in os.listdir(base_dir):
                    if d.startswith("episode_") and os.path.isdir(os.path.join(base_dir, d)):
                        try:
                            num = int(d.split("_")[1])
                            existing_episodes.append(num)
                        except ValueError: pass
                        
                next_ep_num = max(existing_episodes) + 1 if existing_episodes else 1
                temp_dir = "datasets/temp_episode"
                
                with open(os.path.join(temp_dir, "metadata.json"), 'r') as f:
                    metadata = json.load(f)
                metadata["episode_id"] = next_ep_num
                with open(os.path.join(temp_dir, "metadata.json"), 'w') as f:
                    json.dump(metadata, f, indent=4)
                    
                with open(os.path.join(temp_dir, "data.json"), 'w') as f:
                    json.dump(frames_data, f, indent=4)
                    
                target_dir = os.path.join(base_dir, f"episode_{next_ep_num}")
                shutil.move(temp_dir, target_dir)
                print(f"✅ 大丰收！已成功将本次推理数据存入 {target_dir}，直接可用于下一轮训练。")
            else:
                shutil.rmtree("datasets/temp_episode", ignore_errors=True)
                print("🗑️ 已彻底删除本次推理缓存的数据。")
        else:
            shutil.rmtree("datasets/temp_episode", ignore_errors=True)

        print("✅ 部署脚本已彻底安全关闭。")

if __name__ == "__main__":
    main()