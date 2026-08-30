import os
# 强制绕过代理
os.environ["no_proxy"] = "100.64.142.55"
os.environ["NO_PROXY"] = "100.64.142.55"

import time
import cv2
import numpy as np
import threading
import pyrealsense2 as rs
from motorcontroller import MotorController
from openpi_client import websocket_client_policy

# ==========================================
# 🔑 1. 核心网络与硬件配置
# ==========================================
SERVER_IP = "100.64.142.55"  
SERVER_PORT = 8000
PROMPT = "Pick up the screwdriver and place it into the box."

SN_GLOBAL = "821312060126"  
SN_WRIST  = "121622061691"  
SN_SIDE   = "816612062572"  

# ==========================================
# ⚙️ 2. 控制与平滑策略参数 (轨迹融合模式)
# ==========================================
CONTROL_HZ = 50.0   
CONTROL_INTERVAL = 1.0 / CONTROL_HZ
MAX_STEP_DEG = 0.3  # 进一步收紧限幅，压制反复运动
EMA_ALPHA = 0.8     # 指数平滑

# 【核心参数】
EXECUTE_STEPS = 15     # 每一轮执行 15 步后立即触发下一轮采样
ENSEMBLE_WEIGHT = 0.7  # 融合权重：新轨迹占 70%，旧轨迹残余占 30%

is_running = True
STANDBY_POSITIONS = {1: 180.001, 2: 80.001, 3: -85.995, 4: -130.001, 5: 150.001, 6: 130.0, 7: 50.001}

# ==========================================
# 🔄 3. 线程安全动作缓冲区
# ==========================================
class ActionChunkBuffer:
    def __init__(self):
        self.new_chunk = None
        self.lock = threading.Lock()
        self.has_new = False

    def put(self, chunk):
        with self.lock:
            self.new_chunk = chunk
            self.has_new = True

    def get_and_clear(self):
        with self.lock:
            if self.has_new:
                data = self.new_chunk
                self.has_new = False
                return data
            return None

action_buffer = ActionChunkBuffer()

# ==========================================
# 📷 4. 硬件初始化
# ==========================================
def init_camera_by_sn(sn, name):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(sn)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    pipeline.start(config)
    return pipeline

def get_rgb_frame(pipeline):
    success, frames = pipeline.try_wait_for_frames(timeout_ms=50)
    if not success or not frames.get_color_frame(): return None
    img_bgr = np.asanyarray(frames.get_color_frame().get_data())
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

def init_motors():
    controller = MotorController(interface='socketcan', channel='can0')
    controller.start()
    for i in range(1, 8): controller.add_motor(node_id=i, reduction=36.0)
    return controller

# ==========================================
# 🧵 5. 后台推理线程 (负责在运动中“抢拍”)
# ==========================================
def inference_worker(pipe_g, pipe_w, pipe_s, controller, policy):
    global is_running
    while is_running:
        # 这个线程由主线程通过信号量或简单轮询触发，这里采用简单的空转监听
        time.sleep(0.01)

# ==========================================
# 🚀 6. 主程序：轨迹重叠融合执行器
# ==========================================
def main():
    global is_running
    
    controller = init_motors()
    p_g = init_camera_by_sn(SN_GLOBAL, "全局")
    p_w = init_camera_by_sn(SN_WRIST, "手腕")
    p_s = init_camera_by_sn(SN_SIDE, "侧面")
    
    policy = websocket_client_policy.WebsocketClientPolicy(host=SERVER_IP, port=SERVER_PORT)
    
    print("\n✅ 启动【轨迹重叠融合 (Temporal Ensembling)】模式！")
    
    # 状态记忆
    current_chunk = None       # 当前正在执行的轨迹块
    smoothed_qpos = None
    last_sent_qpos = None

    try:
        while is_running:
            # --- 阶段一：感知与推理 (首次或当前块快耗尽时) ---
            img_g, img_w, img_s = get_rgb_frame(p_g), get_rgb_frame(p_w), get_rgb_frame(p_s)
            curr_qpos = np.array([controller.motors[i].position for i in range(1, 8)])
            
            obs = {"cam_global": img_g, "cam_wrist": img_w, "cam_side": img_s, "state": curr_qpos, "prompt": PROMPT}
            
            try:
                res = policy.infer(obs)
                new_chunk = np.array(res["actions"])
                
                # --- 核心融合逻辑 ---
                if current_chunk is not None:
                    # 将旧轨迹剩下的部分与新轨迹的前部进行融合
                    # 假设我们是在第 EXECUTE_STEPS 步拿到的新数据
                    overlap_len = min(len(new_chunk), len(current_chunk) - EXECUTE_STEPS)
                    if overlap_len > 0:
                        print(f"\n🔗 正在融合轨迹：重叠长度 {overlap_len}")
                        # 线性加权：新轨迹逐渐接管
                        for i in range(overlap_len):
                            new_chunk[i] = ENSEMBLE_WEIGHT * new_chunk[i] + (1 - ENSEMBLE_WEIGHT) * current_chunk[i + EXECUTE_STEPS]
                
                current_chunk = new_chunk
            except Exception as e:
                print(f"\n❌ 推理失败: {e}")
                continue

            # --- 阶段二：执行融合后的前 EXECUTE_STEPS 步 ---
            for i in range(EXECUTE_STEPS):
                if not is_running: break
                loop_start = time.time()
                
                # 安全检查
                hardware_error = False
                for m_id in range(1, 8):
                    if hasattr(controller.motors[m_id], 'status') and controller.motors[m_id].status.over_current:
                        hardware_error = True
                if hardware_error: is_running = False; break

                target_qpos = current_chunk[i]
                
                if smoothed_qpos is None:
                    smoothed_qpos = np.array(target_qpos, dtype=np.float64)
                    last_sent_qpos = curr_qpos.copy()

                # 指数平滑
                smoothed_qpos = EMA_ALPHA * np.array(target_qpos) + (1 - EMA_ALPHA) * smoothed_qpos
                
                # 仿真级限速裁剪 (关键：压制反复运动)
                delta = np.clip(smoothed_qpos - last_sent_qpos, -MAX_STEP_DEG, MAX_STEP_DEG)
                safe_qpos = last_sent_qpos + delta
                
                for m_id in range(1, 8):
                    controller.motors[m_id].set_position(safe_qpos[m_id-1])
                
                last_sent_qpos = safe_qpos.copy()
                
                # 50Hz 维护
                time.sleep(max(0, CONTROL_INTERVAL - (time.time() - loop_start)))
                print(f"\r  -> 融合步 {i+1}/{EXECUTE_STEPS} | 关节1:{safe_qpos[0]:5.1f}°", end="")

    except KeyboardInterrupt:
        is_running = False
    finally:
        # 安全归位
        print("\n🔄 归位中...")
        for i in range(1, 8): controller.motors[i].set_position(STANDBY_POSITIONS[i])
        time.sleep(2)
        controller.stop()
        try: p_g.stop(); p_w.stop(); p_s.stop()
        except: pass
        print("✅ 退出成功。")

if __name__ == "__main__":
    main()