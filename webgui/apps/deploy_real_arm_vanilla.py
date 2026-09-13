from core.paths import MOTORS_CONFIG, METRICS_FILE, prepare_runtime

prepare_runtime()
import os
os.environ["no_proxy"] = "100.64.142.55"
os.environ["NO_PROXY"] = "100.64.142.55"

import time
import cv2
import numpy as np
import yaml
import threading
import queue
import json
import pyrealsense2 as rs
from core.motorcontroller import MotorController
from openpi_client import msgpack_numpy
from openpi_client import websocket_client_policy
import websockets.sync.client

# ==========================================
# 🔑 1. 核心网络与硬件配置
# ==========================================
SERVER_IP = "100.64.142.55"  
SERVER_PORT = 8000
PROMPT = "Pick up the screwdriver and place it into the nearby express box."

SN_GLOBAL, SN_WRIST = "821312060126", "816612062572"

# ==========================================
# ⚙️ 2. Pi0 动作块执行参数
# ==========================================
EXECUTION_HZ = 50.0
EXEC_INTERVAL = 1.0 / EXECUTION_HZ
PREFETCH_ACTIONS = 3
MAX_TOTAL_STEPS = 9000

STANDBY_POSITIONS = {
    1: 180.0, 2: 80.0, 3: -100.0,
    4: -120.0, 5: 115.0, 6: 140.0, 7: -115.0
}

JOINT_LIMITS = {
    1: (5.0, 340.0),
    2: (10.0, 180.0),
    3: (-180.0, 0.0),
    4: (-230.0, -10.0),
    5: (10.0, 220.0),
    6: (10.0, 280.0),
    7: (-120.0, 0.0),
}

is_running = True

# 全局最新画面缓存
latest_images = {"global": None, "wrist": None}


class ReliableWebsocketClientPolicy(websocket_client_policy.WebsocketClientPolicy):
    def _wait_for_server(self):
        headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
        connection = websockets.sync.client.connect(
            self._uri,
            compression=None,
            max_size=None,
            additional_headers=headers,
            open_timeout=3,
            ping_interval=20,
            ping_timeout=60,
            close_timeout=2,
        )
        metadata = msgpack_numpy.unpackb(connection.recv())
        return connection, metadata

# ==========================================
# 📷 3. 硬件初始化与线程解耦读取
# ==========================================
def init_camera_by_sn(sn, name):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(sn)
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
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

def init_motors():
    controller = MotorController(interface='socketcan', channel='can0')
    controller.start()
    try:
        with open(MOTORS_CONFIG, 'r') as f:
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


def inference_worker(request_queue, result_queue):
    """在主控制循环请求时推理下一块 Pi0 动作。"""
    global is_running

    policy = None
    last_connection_error = None

    while is_running:
        try:
            obs = request_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        if policy is None:
            try:
                policy = ReliableWebsocketClientPolicy(host=SERVER_IP, port=SERVER_PORT)
                last_connection_error = None
                print("\n✅ 成功连接到 Pi0 推理服务器！")
            except Exception as e:
                error = str(e)
                if error != last_connection_error:
                    print(f"\n⏳ 正在等待/重连 Pi0 推理服务器... ({error})")
                    last_connection_error = error
                try:
                    result_queue.put_nowait(("error", error))
                except queue.Full:
                    pass
                time.sleep(1.0)
                continue

        try:
            infer_start = time.time()
            result = policy.infer(obs)
            infer_time = time.time() - infer_start

            action_chunk = np.asarray(result["actions"], dtype=np.float32)
            if action_chunk.ndim != 2 or action_chunk.shape[1] != 7:
                raise RuntimeError(f"Pi0 返回异常 action shape: {action_chunk.shape}")
            if not np.all(np.isfinite(action_chunk)):
                raise RuntimeError("Pi0 返回动作包含 NaN/Inf")

            result_queue.put(("ok", action_chunk, infer_time))
        except Exception as e:
            if is_running:
                print(f"\n❌ 推理通信异常: {e}")
            policy = None
            try:
                result_queue.put_nowait(("error", str(e)))
            except queue.Full:
                pass

# ==========================================
# 🚀 4. 主程序
# ==========================================
def main():
    global is_running
    
    controller = init_motors()
    p_g = init_camera_by_sn(SN_GLOBAL, "全局")
    p_w = init_camera_by_sn(SN_WRIST, "手腕")
    
    time.sleep(2)
    print("\n✅ VLA 动作部署启动！")
    
    cam_thread = threading.Thread(target=camera_worker, args=(p_g, p_w))
    cam_thread.daemon = True
    cam_thread.start()

    request_queue = queue.Queue(maxsize=1)
    result_queue = queue.Queue(maxsize=1)
    brain_thread = threading.Thread(
        target=inference_worker,
        args=(request_queue, result_queue),
        daemon=True,
    )
    brain_thread.start()
    
    home_arr = np.array([STANDBY_POSITIONS[i] for i in range(1, 7)])
    home_counter = 0
    HOME_THRESHOLD = 12 
    has_left_home = False  
    total_step = 0
    step_limit_failed = False
    chunk_number = 0
    action_chunk = None
    chunk_step = 0
    latest_infer_time = 0.0
    inference_requested = False
    waiting_for_action_logged = False
    np.set_printoptions(precision=2, suppress=True, linewidth=120)
    
    try:
        while is_running:
            if total_step >= MAX_TOTAL_STEPS:
                step_limit_failed = True
                print(
                    f"\n\n❌ 已达到最大执行步数 {MAX_TOTAL_STEPS}，"
                    "本次任务自动判定失败。"
                )
                break

            if action_chunk is None and not inference_requested:
                img_g = latest_images["global"]
                img_w = latest_images["wrist"]
                if img_g is not None and img_w is not None:
                    curr_qpos = np.array(
                        [controller.motors[i].position for i in range(1, 8)]
                    )
                    request_queue.put(
                        {
                            "cam_global": img_g.copy(),
                            "cam_wrist": img_w.copy(),
                            "state": curr_qpos,
                            "prompt": PROMPT,
                        }
                    )
                    inference_requested = True

            if action_chunk is None or chunk_step >= len(action_chunk):
                try:
                    result = result_queue.get_nowait()
                    if result[0] == "ok":
                        _, action_chunk, latest_infer_time = result
                        chunk_step = 0
                        chunk_number += 1
                        inference_requested = False
                        waiting_for_action_logged = False
                    else:
                        print(f"\n⏳ Pi0 推理暂不可用: {result[1]}")
                        action_chunk = None
                        inference_requested = False
                except queue.Empty:
                    pass

            if action_chunk is None or chunk_step >= len(action_chunk):
                if not waiting_for_action_logged:
                    print("\n⏳ 正在等待最新 Pi0 动作块...")
                    waiting_for_action_logged = True
                time.sleep(0.01)
                continue

            if not inference_requested and chunk_step >= max(
                0, len(action_chunk) - PREFETCH_ACTIONS
            ):
                img_g = latest_images["global"]
                img_w = latest_images["wrist"]
                if img_g is not None and img_w is not None:
                    curr_qpos = np.array(
                        [controller.motors[i].position for i in range(1, 8)]
                    )
                    request_queue.put(
                        {
                            "cam_global": img_g.copy(),
                            "cam_wrist": img_w.copy(),
                            "state": curr_qpos,
                            "prompt": PROMPT,
                        }
                    )
                    inference_requested = True

            loop_start = time.time()
            target = action_chunk[chunk_step]
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
                
            safe_target = target.copy()
                
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
                lower, upper = JOINT_LIMITS[m_id]
                safe_target[m_id - 1] = np.clip(
                    safe_target[m_id - 1], lower, upper
                )
                
            for m_id in range(1, 8):
                controller.motors[m_id].set_position(safe_target[m_id-1])
                
            print(
                f"\r[Pi0 {EXECUTION_HZ:.1f}Hz] 块 {chunk_number:4d} "
                f"| 批内 {chunk_step + 1:2d}/{len(action_chunk):2d} "
                f"| 推理 {latest_infer_time:.3f}s | 总步数 {total_step:5d} "
                f"| 离原点偏差:{diff_from_home:.1f}° | 夹爪:{gripper_state}        ",
                end="",
            )
            chunk_step += 1
            total_step += 1
                
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
        metrics_file = METRICS_FILE
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
        if step_limit_failed:
            ans_success = 'n'
            print(
                f"1. 本次实验已因达到 {MAX_TOTAL_STEPS} 步自动判定失败。"
            )
        else:
            ans_success = input(
                "1. 本次实验是否【成功完成】抓取与放置？(y/n): "
            ).strip().lower()
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

        print("✅ 部署脚本已彻底安全关闭。")

if __name__ == "__main__":
    main()
