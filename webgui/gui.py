from nicegui import ui, app
from motorcontroller import MotorController
from queue import Queue
import threading
import time
import yaml 
import json
from datetime import datetime
import asyncio
from fastapi.responses import StreamingResponse
import cv2
from collections import deque
import struct
import can

# === 尝试导入相机所需依赖 ===
try:
    import pyrealsense2 as rs
    import numpy as np
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False

# 在全局变量部分添加以下变量
is_recording = False
recorded_data = []
recording_thread = None
recording_file = "motor_positions.json"

# === 多相机 & 四通道(拼图)全局变量 ===
pipelines = []
camera_running = False
capture_threads = []
# 存储格式变更为：{ 0: bytes, 1: bytes, 2: bytes }，每个 bytes 是一张 4-in-1 的拼图
latest_frame_bytes = {}  
# 独立存储每个相机的实时温度
latest_camera_temps = {}

# === 实时数据曲线图表 全局变量 ===
MAX_HISTORY = 60  # 保持最近 60 个数据点 (大约 30 秒)
chart_timestamps = deque(maxlen=MAX_HISTORY)
chart_data = {
    'position': {i: deque(maxlen=MAX_HISTORY) for i in range(1, 8)},
    'velocity': {i: deque(maxlen=MAX_HISTORY) for i in range(1, 8)},
    'torque':   {i: deque(maxlen=MAX_HISTORY) for i in range(1, 8)}
}

def create_echart_options(title, y_axis_name):
    """辅助函数：生成 ECharts 图表配置项"""
    return {
        'title': {'text': title, 'left': 'center', 'textStyle': {'color': '#333'}},
        'tooltip': {'trigger': 'axis'},
        'legend': {'data': [f'Motor {i}' for i in range(1, 8)], 'bottom': 0},
        'grid': {'left': '5%', 'right': '5%', 'bottom': '15%', 'containLabel': True},
        'xAxis': {'type': 'category', 'boundaryGap': False, 'data': []},
        'yAxis': {'type': 'value', 'name': y_axis_name},
        'series': [{'name': f'Motor {i}', 'type': 'line', 'smooth': True, 'showSymbol': False, 'data': []} for i in range(1, 8)]
    }

def camera_capture_loop(cam_idx, pipeline, align):
    """独立的相机采集线程，处理丢帧并生成 4-in-1 拼图流，同时读取温度"""
    global camera_running, latest_frame_bytes, latest_camera_temps
    
    W_MAIN, H_MAIN = 640, 480
    
    # 提前获取深度传感器对象，避免在 while 循环中频繁查询降低性能
    depth_sensor = None
    try:
        active_dev = pipeline.get_active_profile().get_device()
        sensors = active_dev.query_sensors()
        depth_sensor = next((s for s in sensors if s.is_depth_sensor()), None)
    except Exception as e:
        print(f"获取相机 {cam_idx} 传感器失败: {e}")
    
    while camera_running and pipeline:
        try:
            # 读取芯片温度
            if depth_sensor and depth_sensor.supports(rs.option.asic_temperature):
                latest_camera_temps[cam_idx] = depth_sensor.get_option(rs.option.asic_temperature)

            # 读取并对齐图像帧
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            aligned_frames = align.process(frames) if align else frames
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            ir_left_frame = frames.get_infrared_frame(1) 
            ir_right_frame = frames.get_infrared_frame(2)

            # 预设全黑的占位画布，防止 USB 丢帧导致整个相机黑屏
            color_image = np.zeros((H_MAIN, W_MAIN, 3), dtype=np.uint8)
            depth_colormap = np.zeros((H_MAIN, W_MAIN, 3), dtype=np.uint8)
            ir_left_resized = np.zeros((H_MAIN, W_MAIN, 3), dtype=np.uint8)
            ir_right_resized = np.zeros((H_MAIN, W_MAIN, 3), dtype=np.uint8)

            # 1. 提取彩色图
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
            
            # 2. 提取深度图并转换为伪彩
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 3. 提取左红外
            if ir_left_frame:
                ir_left_raw = np.asanyarray(ir_left_frame.get_data())
                ir_left_bgr = cv2.cvtColor(ir_left_raw, cv2.COLOR_GRAY2BGR)
                ir_left_resized = cv2.resize(ir_left_bgr, (W_MAIN, H_MAIN))
                
            # 4. 提取右红外
            if ir_right_frame:
                ir_right_raw = np.asanyarray(ir_right_frame.get_data())
                ir_right_bgr = cv2.cvtColor(ir_right_raw, cv2.COLOR_GRAY2BGR)
                ir_right_resized = cv2.resize(ir_right_bgr, (W_MAIN, H_MAIN))

            # 构建 2x2 拼图矩阵 (Top: Color | Depth, Bottom: IR L | IR R)
            top_row = np.hstack((color_image, depth_colormap))
            bottom_row = np.hstack((ir_left_resized, ir_right_resized))
            grid = np.vstack((top_row, bottom_row))
            
            # 增加各通道的英文标识
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(grid, 'Color', (20, 40), font, 1.2, (255, 255, 255), 2)
            cv2.putText(grid, 'Depth', (W_MAIN + 20, 40), font, 1.2, (255, 255, 255), 2)
            cv2.putText(grid, 'IR Left', (20, H_MAIN + 40), font, 1.2, (255, 255, 255), 2)
            cv2.putText(grid, 'IR Right', (W_MAIN + 20, H_MAIN + 40), font, 1.2, (255, 255, 255), 2)

            # 将巨大的 1280x960 拼图缩小一半为 640x480
            grid_small = cv2.resize(grid, (0, 0), fx=0.5, fy=0.5)

            # 压缩为单张 JPEG (每台相机只占用一个连接)
            ret, buffer = cv2.imencode('.jpg', grid_small, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
            if ret:
                latest_frame_bytes[cam_idx] = buffer.tobytes()
                
        except Exception as e:
            time.sleep(0.01)  

async def generate_frames(cam_idx):
    """FastAPI 异步生成器：为单个相机推送 4-in-1 的视频流 (极限 60FPS 推流)"""
    global camera_running, latest_frame_bytes
    try:
        while camera_running:
            frame = latest_frame_bytes.get(cam_idx)
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            await asyncio.sleep(0.016)  
    except asyncio.CancelledError:
        pass
    except Exception:
        pass

# === 注册独立的拼图视频流后端路由 ===
@app.get('/video_feed/{cam_idx}')
async def video_feed(cam_idx: int):
    return StreamingResponse(generate_frames(cam_idx), media_type='multipart/x-mixed-replace; boundary=frame')

def start_camera():
    global pipelines, camera_running, capture_threads, latest_frame_bytes, latest_camera_temps
    if not HAS_REALSENSE:
        ui.notify("Missing pyrealsense2 or opencv-python! Please pip install them.", type='negative')
        return
    if camera_running:
        ui.notify("Cameras are already running!")
        return
    
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        num_devices = len(devices)
        if num_devices == 0:
            ui.notify("No RealSense cameras detected on USB!", type='negative')
            return

        pipelines = []
        capture_threads = []
        latest_frame_bytes = {}
        latest_camera_temps = {}
        camera_running = True
        
        MAX_CAMERAS = 3
        align = rs.align(rs.stream.color)
        
        # 遍历并启动所有检测到的相机
        for i, dev in enumerate(devices):
            if i >= MAX_CAMERAS: 
                break 
                
            sn = dev.get_info(rs.camera_info.serial_number)
            pipeline = rs.pipeline(ctx)
            config = rs.config()
            config.enable_device(sn)
            
            # === 极限压力测试：强制 12 通道全部 640x480 @ 60FPS ===
            W_MAIN, H_MAIN = 640, 480 
            FPS = 60
            config.enable_stream(rs.stream.color, W_MAIN, H_MAIN, rs.format.bgr8, FPS)
            config.enable_stream(rs.stream.depth, W_MAIN, H_MAIN, rs.format.z16, FPS)
            config.enable_stream(rs.stream.infrared, 1, W_MAIN, H_MAIN, rs.format.y8, FPS)
            config.enable_stream(rs.stream.infrared, 2, W_MAIN, H_MAIN, rs.format.y8, FPS)
            
            try:
                # 移除自动降频回退逻辑，直接硬上 60FPS
                if config.can_resolve(pipeline):
                    pipeline.start(config)
                    pipelines.append(pipeline)
                    ui.notify(f"Camera {i+1} started (Force 60FPS)", color='green')
                else:
                    raise RuntimeError("带宽或硬件不支持 60FPS，拒绝启动")
            except Exception as e:
                ui.notify(f"Camera {i+1} start failed: {e}", type='negative')
                continue
            
            # 开启后端处理线程
            t = threading.Thread(target=camera_capture_loop, args=(i, pipeline, align), daemon=True)
            t.start()
            capture_threads.append(t)
            
            # 前端唤醒拉流
            ui.run_javascript(f'try {{ document.getElementById("cam_{i}_feed").src = "/video_feed/{i}?t=" + new Date().getTime(); }} catch(e) {{}}')
            
    except Exception as e:
        ui.notify(f"Failed to start cameras: {e}", type='negative')
        camera_running = False

def stop_camera():
    global pipelines, camera_running, latest_frame_bytes, capture_threads, latest_camera_temps
    if camera_running:
        camera_running = False
        
        MAX_CAMERAS = 3
        for i in range(MAX_CAMERAS):
            ui.run_javascript(f'try {{ document.getElementById("cam_{i}_feed").src = ""; }} catch(e) {{}}')
            ui.run_javascript(f'try {{ document.getElementById("cam_{i}_temp").innerText = "实时温度: 离线"; }} catch(e) {{}}')
            
        time.sleep(0.3)  
        
        for pipe in pipelines:
            try:
                pipe.stop()
            except Exception as e:
                pass
                
        pipelines = []
        capture_threads = []
        latest_frame_bytes = {}
        latest_camera_temps = {}
        ui.notify("All Cameras stopped")

# ================= 修改的示教控制逻辑 (包含第七关节) =================
def start_damping():
    for node_id, motor in controller.motors.items():
        motor.enable()
        time.sleep(0.05)
        motor.set_damping_mode()
    ui.notify("All motors (including Gripper) set to damping mode", color='orange')

def stop_damping():
    for node_id, motor in controller.motors.items():
        motor.set_stop_damping_mode()
        time.sleep(0.05)
        motor.disable() # 退出阻尼后，所有轴失能
    ui.notify("All motors disabled damping mode", color='green')
# ===================================================================

def start_recording():
    global is_recording, recorded_data, recording_thread
    
    if is_recording:
        ui.notify("Recording is already in progress!")
        return
    
    is_recording = True
    recorded_data = []
    ui.notify("Recording started")
    
    def record_positions():
        while is_recording:
            positions = {}
            status_data = []
            for status in controller.get_all_motor_status():
                status_data.append({
                    'node_id': status['node_id'],
                    'position': status['position']
                })
            
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            recorded_data.append({
                'timestamp': timestamp,
                'positions': status_data
            })
            
            time.sleep(0.5)  
    
    recording_thread = threading.Thread(target=record_positions, daemon=True)
    recording_thread.start()

def stop_recording():
    global is_recording
    
    if not is_recording:
        ui.notify("No active recording to stop!")
        return
    
    is_recording = False
    ui.notify("Recording stopped")
    
    try:
        with open(recording_file, 'w') as f:
            json.dump(recorded_data, f, indent=4)
        ui.notify(f"Data saved to {recording_file}")
    except Exception as e:
        ui.notify(f"Error saving data: {str(e)}")

def replay_recording():
    if is_recording:
        ui.notify("Cannot replay while recording!")
        return
    
    try:
        with open(recording_file, 'r') as f:
            data = json.load(f)
    except Exception as e:
        ui.notify(f"Error loading recording: {str(e)}")
        return
    
    if not data:
        ui.notify("No recording data to replay!")
        return
    
    ui.notify(f"Replaying {len(data)} position records")
    
    def replay_task():
        for record in data:
            positions = record['positions']
            for pos in positions:
                motor_id = pos['node_id']
                position = pos['position']
                if motor_id in controller.motors:
                    controller.motors[motor_id].set_position(position)
            time.sleep(0.5)  
    
    threading.Thread(target=replay_task, daemon=True).start()

# Create the motor controller
controller = MotorController()

with open('motors.yaml', 'r') as file:
    motor_config = yaml.safe_load(file)

for node in motor_config['nodes']:
    node_id = node['id']
    reduction = node['reduction']
    try:
        controller.add_motor(node_id, reduction=reduction)
    except Exception as e:
        print(f"Error initializing motor {node_id}: {e}")
        
if controller.is_initialized():
    controller.start()

message_queue = Queue()

def process_messages():
    while not message_queue.empty():
        message = message_queue.get()
        ui.notify(message)

def enable_all_motors():
    for motor in controller.motors.values():
        motor.enable()
        time.sleep(0.05)
    ui.notify("All motors enabled")

homing_event = threading.Event()

def restore_positions():
     for motor in controller.motors.values():
        pos = motor.saved_position
        motor.set_position(pos)

def manual_homing_all_motors():
    if homing_event.is_set():
        ui.notify("Manual homing is already in progress!")
        return
    
    def _manual_homing_task():
        homing_event.set()
        try:
            message_queue.put("Manual homing is on the progress!")
            i = 0
            for motor in controller.motors.values():
                i += 1
                if i == 2:
                    motor.set_position(80)
                elif i == 3:
                    motor.set_position(92)
                elif i == 5:
                    motor.set_position(115)
                else:
                    motor.set_position(0)
            time.sleep(6)
            for motor in controller.motors.values():
                i += 1
                motor.set_home()
            message_queue.put("All motors have set to home position!!!")
        finally:
            homing_event.clear()

    threading.Thread(target=_manual_homing_task, daemon=True).start()

def smart_homing(motor, node_id, homing_current, homing_pos, expected_pos):
    """
    智能调零序列：彻底绕开底层同值死锁 Bug，采用纯软件憋停检测，并阻塞等待返回初始位置
    """
    try:
        message_queue.put(f"[{node_id}] 1. 清洗底层状态机与错误锁死...")
        motor.status.over_current = False
        motor.error_resets()
        time.sleep(0.2)
        motor.disable()
        time.sleep(0.2)

        message_queue.put(f"[{node_id}] 2. 注入防死锁参数 (运行={homing_current}A, 保护=20A)...")
        tx_id = motor.build_can_id(dir_bit=0, cmd_id=17) # CMD_SET_CONFIG = 0x11
        
        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 7, float(homing_current)), is_extended_id=False))
        time.sleep(0.05)
        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 28, 20.0), is_extended_id=False))
        time.sleep(0.05)

        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 23, 5.0), is_extended_id=False))
        time.sleep(0.05)
        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 24, 10.0), is_extended_id=False))
        time.sleep(0.05)
        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 25, 10.0), is_extended_id=False))
        time.sleep(0.05)

        message_queue.put(f"[{node_id}] 3. 强行切换至位置模式并使能...")
        motor.set_stop_damping_mode()
        time.sleep(0.2)
        motor.enable()
        time.sleep(0.3)

        message_queue.put(f"[{node_id}] 4. 下发探零运动指令: {homing_pos}°")
        start_pos = motor.position
        motor.set_position(homing_pos)
        
        start_t = time.time()
        stall_counter = 0
        has_backed_off = False  
        last_ui_update = 0
        
        while True:
            elapsed = time.time() - start_t
            if elapsed > 40:
                message_queue.put(f"[{node_id}] ❌ 调零超时！(可能原因: 电机被卡死未移动，或无法克服静摩擦)")
                return False, "HOMING_FAIL"
            
            try:
                vel = abs(motor.motor_velocity)
                curr = abs(motor.motor_current)
            except AttributeError:
                vel = abs(motor.velocity)
                curr = abs(motor.current)
                
            pos = motor.position
            
            if time.time() - last_ui_update > 0.5:
                message_queue.put(f"[{node_id}] 实时状态 -> 位置: {pos:.1f}°, 速度: {vel:.1f}, 电流: {curr:.2f}A")
                last_ui_update = time.time()
            
            if motor.status.over_current:
                if abs(pos - start_pos) >= 2.0 or has_backed_off:
                    message_queue.put(f"[{node_id}] ⚠️ 触碰限位触发极值过流，视作调零成功！")
                    break
                    
            if not has_backed_off and (elapsed > 2.0 or motor.status.over_current):
                if abs(pos - start_pos) < 2.0:
                    message_queue.put(f"[{node_id}] ⚠️ 发现起步即卡死，执行相对反向解锁...")
                    motor.status.over_current = False
                    motor.disable()
                    time.sleep(0.1)
                    motor.error_resets()
                    time.sleep(0.1)
                    motor.set_stop_damping_mode()
                    time.sleep(0.1)
                    motor.enable()
                    time.sleep(0.2)
                    
                    relative_offset = 30.0 if homing_pos < 0 else -30.0
                    backoff_target = pos + relative_offset
                    motor.set_position(backoff_target)
                    time.sleep(3.0)
                    
                    start_pos = motor.position
                    motor.set_position(homing_pos)
                    start_t = time.time()
                    has_backed_off = True
                    stall_counter = 0
                    continue

            if elapsed > 1.5:
                if vel < 0.4 and abs(pos - start_pos) > 2.0 and curr > (homing_current * 0.6):
                    stall_counter += 1
                else:
                    stall_counter = 0
                    
                if stall_counter >= 15:
                    message_queue.put(f"[{node_id}] ✅ 成功检测到物理限位停转 (持续1.5秒高电流憋停)！")
                    break
                    
            time.sleep(0.1)
            
        message_queue.put(f"[{node_id}] 5. 正在强制锁定零点并恢复原厂输出力气...")
        motor.error_resets()
        time.sleep(0.1)
        motor.disable()
        time.sleep(0.2)
        motor.set_home()
        time.sleep(0.5)
        
        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 28, 12.0), is_extended_id=False))
        time.sleep(0.05)
        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 7, 10.0), is_extended_id=False))
        time.sleep(0.05)
        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 23, 2.0), is_extended_id=False)) 
        time.sleep(0.05)
        motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 24, 5.0), is_extended_id=False)) 
        time.sleep(0.05)
        
        motor.enable()
        time.sleep(0.2)
        
        message_queue.put(f"[{node_id}] 6. 正在优雅回退到安全姿态: {expected_pos}°")
        motor.set_position(expected_pos)
        
        reach_timeout = 30.0  
        start_backoff_t = time.time()
        reached = False
        last_ui_update = 0
        
        while True:
            elapsed_backoff = time.time() - start_backoff_t
            if elapsed_backoff > reach_timeout:
                break
                
            current_pos = motor.position
            diff = abs(current_pos - expected_pos)
            
            if time.time() - last_ui_update > 0.5:
                message_queue.put(f"[{node_id}] 回退监控 -> 当前位置: {current_pos:.1f}°, 目标: {expected_pos:.1f}° (偏差: {diff:.1f}°)")
                last_ui_update = time.time()
            
            if diff < 1.5:
                reached = True
                break
            time.sleep(0.1)
            
        if reached:
            message_queue.put(f"[{node_id}] 🎉 关节已精准到达安全姿态，调零圆满完成。")
            return True, "success"
        else:
            message_queue.put(f"[{node_id}] ❌ 致命错误：回退动作超时！未能到达安全姿态。")
            return False, "BACKOFF_FAIL"
    except Exception as e:
        return False, str(e)


def homing_all_motors():
    """完全复刻 auto_homing.py 的调度执行链"""
    if homing_event.is_set():
        ui.notify("Homing is already in progress!")
        return
        
    def _homing_task():
        homing_event.set()
        try:
            homing_params = {
                1: (3.0, -360, 180),
                2: (4.0, -360, 81),
                3: (4.0, 360, -85),
                4: (2.0, 360, -130),
                5: (2.0, -360, 110),
                6: (1.0, -180, 130), 
                7: (1.0, -120, 0)
            }
            final_target_positions = {
                1: 180.001,
                2: 80.001,
                3: -85.995,
                4: -130.001,
                5: 150.001,
                6: 130.0,
                7: 50.001
            }

            # ---------------- [1/3] 第五关节优先调零 ----------------
            message_queue.put("\n[1/3] 预动作：仅使能第五关节进行寻零，并停留在零位...")
            motor5 = controller.motors.get(5)
            if motor5:
                params5 = homing_params.get(5)
                if params5:
                    success, msg = smart_homing(motor5, 5, params5[0], params5[1], 0.0)
                    if msg == "BACKOFF_FAIL":
                        time.sleep(2)
                        success, msg = smart_homing(motor5, 5, params5[0], params5[1], 0.0)
                    if not success:
                        message_queue.put("⚠️ 第五关节预先调零失败，已安全终止。")
                        return

            # ---------------- [2/3] 按序执行 1-7 碰撞调零 ----------------
            message_queue.put("\n[2/3] 严格按序执行智能碰撞调零 (第4个结束后第5个直接移动)...")
            for i in range(1, 8):
                motor = controller.motors.get(i)
                if not motor: continue
                    
                if i == 5:
                    target_pos = final_target_positions.get(5, 135.001)
                    message_queue.put(f"\n---> [5] 第四关节已结束，第五关节无需调零，直接移动至待机位置: {target_pos}° ...")
                    motor.set_position(target_pos)
                    
                    reach_timeout = 40.0
                    start_t = time.time()
                    reached = False
                    last_ui_update = 0
                    while True:
                        if time.time() - start_t > reach_timeout: break
                        curr_pos = motor.position
                        diff = abs(curr_pos - target_pos)
                        
                        if time.time() - last_ui_update > 0.5:
                            message_queue.put(f"     Motor [5] 移动监控 -> 当前: {curr_pos:.1f}°, 目标: {target_pos:.1f}°")
                            last_ui_update = time.time()
                            
                        if diff < 1.5:
                            reached = True
                            break
                        time.sleep(0.1)
                        
                    if not reached:
                        message_queue.put(f"     ⚠️ 警告: Motor [{i}] 移动超时，未能精准到达最终位置！")
                        return
                    else:
                        message_queue.put(f"     ✅ Motor [{i}] 已安全就位。")
                    continue

                params = homing_params.get(i)
                if params:
                    success, msg = smart_homing(motor, i, params[0], params[1], params[2])
                    
                    if msg == "BACKOFF_FAIL":
                        message_queue.put(f"\n⚠️ 触发容错机制：Motor [{i}] 回退不到位，可能因偶发摩擦力卡住。")
                        message_queue.put(f"   正在冷静 2 秒后，对该关节重新执行一遍完整的自动调零...")
                        time.sleep(2)
                        success, msg = smart_homing(motor, i, params[0], params[1], params[2])
                        
                    if not success:
                        message_queue.put("\n⚠️ 调零或回退最终失败，为防止机械臂干涉，已安全终止后续关节。")
                        return

            # ---------------- [3/3] 执行全局最终待机姿态 ----------------
            message_queue.put("\n[3/3] 正在顺序将所有关节移动至目标全局待机姿态...")
            for i in range(1, 8):
                motor = controller.motors.get(i)
                if not motor: continue
                
                target = final_target_positions.get(i)
                if target is not None:
                    message_queue.put(f"\n---> 正在控制 Motor [{i}] 移动至最终位置: {target}° ...")
                    motor.set_position(target)
                    
                    reach_timeout = 40.0
                    start_t = time.time()
                    reached = False
                    last_ui_update = 0
                    
                    while True:
                        if time.time() - start_t > reach_timeout: break
                        curr_pos = motor.position
                        diff = abs(curr_pos - target)
                        
                        if time.time() - last_ui_update > 0.5:
                            message_queue.put(f"     Motor [{i}] 移动监控 -> 当前: {curr_pos:.1f}°, 目标: {target:.1f}°")
                            last_ui_update = time.time()
                            
                        if diff < 1.5:
                            reached = True
                            break
                        time.sleep(0.1)
                        
                    if not reached:
                        message_queue.put(f"     ⚠️ 警告: Motor [{i}] 移动超时，未能精准到达最终位置！")
                        return
                    else:
                        message_queue.put(f"     ✅ Motor [{i}] 已安全就位。")
                        time.sleep(0.5)
                        
            message_queue.put("\n=====================================================")
            message_queue.put("✅ 所有调零与最终姿态归位流程完美结束！")
            message_queue.put("=====================================================")
        except Exception as e:
            message_queue.put(f"Homing exception: {e}")
        finally:
            homing_event.clear()

    threading.Thread(target=_homing_task, daemon=True).start()

def get_all_motors_status():
    for motor in controller.motors.values():
        motor.reference_status()
        motor.reference_saved_position()
        motor.reference_value1()
        time.sleep(0.02)  

def disable_all_motors():
    for motor in controller.motors.values():
        motor.disable()
        time.sleep(0.05)  
    ui.notify("All motors disabled")

def clear_up_errors():
    for motor in controller.motors.values():
        motor.error_resets()
        time.sleep(0.05) 
    ui.notify("All motors error resets")

def move_motors(position: float):
    for motor in controller.motors.values():
        motor.set_position(position)
    ui.notify(f"All motors moving to position {position}")

def move_motors_ready(position: float):
    i = 0
    for motor in controller.motors.values():
        i += 1
        if i == 2:
            motor.set_position(-80)
        elif i == 3:
            motor.set_position(-90)
        elif i == 5:
            motor.set_position(0)
        else:
            motor.set_position(0)
    ui.notify(f"All motors moving to position {position}")

def move_motors_range(position: float):
    i = 0
    for motor in controller.motors.values():
        i += 1
        if i == 2:
            motor.set_position(90)
        elif i == 3:
            motor.set_position(90)
        else:
            motor.set_position(0)
    ui.notify(f"All motors moving to long range {position}")

def loop_motors(position: float):
    if homing_event.is_set():
        ui.notify("Looping is already in progress!")
        return
    def _loop_task():
        homing_event.set()
        try:
            for _ in range(10):    
                for motor in controller.motors.values():
                    motor.set_position(position * -1)
                time.sleep(7)
                for motor in controller.motors.values():
                    motor.set_position(position)
                time.sleep(7)
        finally:
            homing_event.clear()
    threading.Thread(target=_loop_task, daemon=True).start()
    ui.notify(f"All motors moving to position {position}")

def move_motors_home(position: float):
    i = 0
    for motor in controller.motors.values():
        motor.set_position(position * 0)

    ui.notify(f"All motors moving to home position {position}")

calibration_progress = 0  
calibration_in_progress = False

def run_calibration_for_motor(target_node_id):
    global calibration_progress, calibration_in_progress
    
    if calibration_in_progress:
        ui.notify("Calibration is already in progress!")
        return
    
    calibration_in_progress = True
    calibration_progress = 0
    progress_bar.visible = True
    
    def _calibration_task():
        global calibration_progress
        global calibration_in_progress
        try:
            motor = controller.motors.get(target_node_id)
            if not motor:
                message_queue.put(f"Error: Motor Node [{target_node_id}] not found in controller!")
                return
                
            message_queue.put(f"Preparing Motor [{target_node_id}] for calibration...")
            
            motor.disable()
            time.sleep(0.5)
            motor.error_resets()
            time.sleep(0.5)
            motor.disable()
            time.sleep(0.5)
            
            motor.calibration_progress = 0
            calibration_progress = 0
            
            message_queue.put(f"Sending CALIB_START command to Motor [{target_node_id}]...")
            motor.start_calibration()
            
            start_time = time.time()
            while motor.calibration_progress < 137:
                if time.time() - start_time > 60:
                    message_queue.put(f"Motor [{target_node_id}] Calibration Timeout (60s)!")
                    break
                time.sleep(0.1)
                calibration_progress = motor.calibration_progress
                
            time.sleep(1)
        finally:
            message_queue.put(f"Motor [{target_node_id}] Calibration process finished.")
            calibration_in_progress = False
    
    threading.Thread(target=_calibration_task, daemon=True).start()

def move_motors_neg(position: float):
    for motor in controller.motors.values():
        motor.set_position(position * -1)
    ui.notify(f"All motors moving to position {position}")

def update_status_display():
    process_messages()
        
    status_data = []
    
    # 提取所有当前电机状态，用于更新表格和曲线图
    all_status = controller.get_all_motor_status()
    
    # === 同步更新实时曲线图表数据 ===
    current_time = datetime.now().strftime("%H:%M:%S")
    chart_timestamps.append(current_time)
    
    # 建立快捷查找字典
    status_dict = {s['node_id']: s for s in all_status}
    
    for i in range(1, 8):
        if i in status_dict:
            chart_data['position'][i].append(status_dict[i]['position'])
            chart_data['velocity'][i].append(status_dict[i]['velocity'])
            chart_data['torque'][i].append(status_dict[i]['torque'])
        else:
            # 如果某台电机没在线，填充 0 或者保持最后一次数值防止报错
            last_pos = chart_data['position'][i][-1] if chart_data['position'][i] else 0.0
            chart_data['position'][i].append(last_pos)
            chart_data['velocity'][i].append(0.0)
            chart_data['torque'][i].append(0.0)
            
    # 推送数据到网页 ECharts
    try:
        # 更新时间 X 轴
        pos_chart.options['xAxis']['data'] = list(chart_timestamps)
        vel_chart.options['xAxis']['data'] = list(chart_timestamps)
        torq_chart.options['xAxis']['data'] = list(chart_timestamps)
        
        # 更新具体线条数据
        for i in range(1, 8):
            pos_chart.options['series'][i-1]['data'] = list(chart_data['position'][i])
            vel_chart.options['series'][i-1]['data'] = list(chart_data['velocity'][i])
            torq_chart.options['series'][i-1]['data'] = list(chart_data['torque'][i])
            
        pos_chart.update()
        vel_chart.update()
        torq_chart.update()
    except Exception:
        pass # UI 未就绪时静默忽略

    # === 原有的状态表格数据整理 ===
    for status in all_status:
        errors = [k for k, v in status['errors'].items() if v]
        if calibration_in_progress:
            progress_bar.value = calibration_progress / 137.0  

        status_data.append({
            'node_id': status['node_id'],
            'enabled': 'Yes' if status['enabled'] else 'No',
            'target_reached': 'Yes' if status['target_reached'] else 'No',
            'errors': ', '.join(errors) if errors else 'None',
            'last_update': f"{status['last_update']:.1f}s ago",
            'position':  f"{status['position']:.3f}",
            'saved_position':  f"{status['saved_position']:.3f}",
            'velocity':  f"{status['velocity']:.3f}",
            'torque':  f"{status['torque']:.3f}",
            'current':  f"{status['current']:.3f}",
            'bus_current':  f"{status['bus_current']:.3f}",
            'bus_voltage':  f"{status['bus_voltage']:.3f}",
            'motor_power':  f"{status['motor_power']:.3f}"
        })
    status_table.rows = status_data
    status_table.update()
    
    # === 同步更新前端界面上的相机温度 ===
    global latest_camera_temps
    for i, temp in latest_camera_temps.items():
        ui.run_javascript(f'try {{ document.getElementById("cam_{i}_temp").innerText = "实时温度: {temp:.1f} °C"; }} catch(e) {{}}')

# NiceGUI App
app.title = "Motor Control Panel"

def show_confirm_dialog():
    with ui.dialog() as dialog:  
        with ui.card():
            ui.label("Please Select a Motor to Calibrate:")
            with ui.row().classes('flex flex-wrap gap-2'):
                for n_id in range(1, 8):
                    ui.button(f"Motor {n_id}", on_click=lambda _, n=n_id: (run_calibration_for_motor(n), dialog.close()))
                ui.button("Cancel", on_click=dialog.close, color='red')  
    dialog.open() 

with ui.row().classes('w-full'):
    with ui.tabs() as tabs:
        menu1 = ui.tab('Motor Control')
        menu2 = ui.tab('Settings')
        menu3 = ui.tab('Camera Vision')
        menu4 = ui.tab('Live Charts')  # === 新增：图表标签页 ===

with ui.tab_panels(tabs, value=menu1).classes('w-full'):
    
    # === Menu 1: Motor Control ===
    with ui.tab_panel(menu1):
        with ui.card().classes('w-full'):
            ui.label('Motor Control Panel').classes('text-h4')
            
            columns = [
                {'name': 'node_id', 'label': 'Motor ID', 'field': 'node_id'},
                {'name': 'enabled', 'label': 'Enabled', 'field': 'enabled'},
                {'name': 'target_reached', 'label': 'Target Reached', 'field': 'target_reached'},
                {'name': 'errors', 'label': 'Errors', 'field': 'errors'},
                {'name': 'last_update', 'label': 'Last Update', 'field': 'last_update'},
                {'name': 'position', 'label': 'Position', 'field': 'position'},
                {'name': 'saved_position', 'label': 'Saved Position', 'field': 'saved_position'},
                {'name': 'velocity', 'label': 'Velocity', 'field': 'velocity'},
                {'name': 'torque', 'label': 'Torque', 'field': 'torque'},
                {'name': 'current', 'label': 'Current', 'field': 'current'},
                {'name': 'bus_current', 'label': 'Bus Current', 'field': 'bus_current'},
                {'name': 'bus_voltage', 'label': 'Bus Voltage', 'field': 'bus_voltage'},
                {'name': 'motor_power', 'label': 'Motor Power', 'field': 'motor_power'}
            ]
            status_table = ui.table(columns=columns, rows=[]).classes('w-full')
            
            with ui.row().classes('w-full items-center'):
                position_input = ui.number(label='Position (turns)', value=0.0).classes('w-32')
                ui.button('+', on_click=lambda: move_motors(position_input.value))
                ui.button('Home', on_click=lambda: move_motors_home(position_input.value))
                ui.button('-', on_click=lambda: move_motors_neg(position_input.value))
                ui.button('Ready', on_click=lambda: move_motors_ready(position_input.value))
                ui.button('Max. Range', on_click=lambda: move_motors_range(position_input.value))
                ui.button('Loop', on_click=lambda: loop_motors(position_input.value))
            
            ui.separator().classes('flex-grow')  
            ui.label('Control').classes('text-sm text-gray-500') 
            with ui.row().classes('w-full items-center'):
                ui.button('Enable All Motors', on_click=enable_all_motors, color='green')
                ui.button('Disable All Motors', on_click=disable_all_motors, color='red')
                ui.button('Reset Errors', on_click=clear_up_errors, color='blue')
                ui.button('RESTORE POSITIONS', on_click=restore_positions, color='blue')

            ui.separator().classes('flex-grow')  
            ui.label('ARM Calibration').classes('text-sm text-gray-500') 
            with ui.row().classes('w-full items-center'):
                ui.button('HOMING', on_click=manual_homing_all_motors, color='green')
                ui.button('AUTO HOMING', on_click=homing_all_motors, color='green')
            
            ui.separator().classes('flex-grow')  
            ui.label('Teaching & Gripper').classes('text-sm text-gray-500') 
            with ui.row().classes('w-full items-center'):
                ui.button('Damping', on_click=start_damping, color='red')
                ui.button('STOP Damping', on_click=stop_damping, color='green')
                ui.button('Open Gripper', on_click=lambda: controller.motors[7].set_position(50.0) if 7 in controller.motors else None, color='teal')
                ui.button('Close Gripper', on_click=lambda: controller.motors[7].set_position(0.0) if 7 in controller.motors else None, color='teal')
                ui.button('Record', on_click=start_recording, color='orange')
                ui.button('Stop', on_click=stop_recording, color='red')
                ui.button('Replay', on_click=replay_recording, color='blue')
            
            ui.separator().classes('flex-grow')  
            ui.label('Motor Calibration').classes('text-sm text-gray-500') 
            with ui.row().classes('w-full items-center'):
                ui.button('Calibration Selector', on_click=show_confirm_dialog, color='blue')
                progress_bar = ui.linear_progress(value=0, color='green').classes('w-64')  
                progress_bar.visible = False  

    # === Menu 2: Settings ===
    with ui.tab_panel(menu2):
        with ui.card().classes('w-full'):
            ui.label('Settings').classes('text-h4')

            with ui.tabs().classes('w-full') as motor_tabs:
                for node_id in range(1, 8):
                    ui.tab(f'Motor {node_id}')

            with ui.tab_panels(motor_tabs, value='MOTOR').classes('w-full'):
                for node_id in range(1, 8):
                    with ui.tab_panel(f'Motor {node_id}'):
                        with ui.card().classes('w-full'):
                            ui.label(f'Motor {node_id}').classes('text-h6')
                            pos_input = ui.number(label=f'Position', value=0.0).classes('w-32')
                            ui.button(f'Move', on_click=lambda _, n=node_id, p=pos_input: controller.motors[n].set_position(p.value))
                            with ui.expansion('Motor Parameters', icon='menu').classes('w-full'):
                                with ui.row().classes('w-full items-center'):
                                    current_limit = ui.number(label=f'Current Limit', value=3).bind_value(controller.motors[node_id].configs.current_limit, 'current_limit')
                            with ui.expansion('Controller Parameters', icon='menu').classes('w-full'):
                                with ui.row().classes('w-full items-center'):
                                    config_node_id = ui.number(label=f'Node Id', value=3).bind_value(controller.motors[node_id].configs.nodeid, 'node_id')
                                    config_mode = ui.number(label=f'Control Mode', value=3).tooltip("3 = profiled position mode").bind_value(controller.motors[node_id].configs.control_mode, 'control_mode')
                                    velocity = ui.number(label=f'Velocity', value=3.0).tooltip("normal=3 middle=7 fast=15 crazzy=30").bind_value(controller.motors[node_id].configs.velocity, 'velocity')
                                    acceleration = ui.number(label=f'Acceleration', value=5.0).tooltip("normal=4.8 middle=11 fast=24 crazzy=50").bind_value(controller.motors[node_id].configs.acceleration, 'acceleration')
                                    deceleration = ui.number(label=f'Deceleration', value=5.0).tooltip("normal=4.8 middle=11 fast=24 crazzy=50").bind_value(controller.motors[node_id].configs.deceleration, 'deceleration')
                                    ui.separator().classes('flex-grow')  
                                    config_kp_gain = ui.number(label=f'KP Gain', value=60.0).tooltip("value: 0 - 1000").bind_value(controller.motors[node_id].configs.kp_gain, 'kp_gain')
                                    config_kd_gain = ui.number(label=f'KD Gain', value=0.003, format='%.6f', step=0.0001).tooltip("value: 0 - 1000").bind_value(controller.motors[node_id].configs.kd_gain, 'kd_gain')
                                    config_ki_gain = ui.number(label=f'KI Gain', value=0.0001, format='%.6f', step=0.000001).tooltip("value: 0 - 1000").bind_value(controller.motors[node_id].configs.ki_gain, 'ki_gain')
                            with ui.expansion('Security Parameters', icon='menu').classes('w-full'):
                                with ui.row().classes('w-full items-center'):
                                    over_current = ui.number(label=f'Motor Over current', value=4).bind_value(controller.motors[node_id].configs.protect_over_current, 'protect_over_current')
                            ui.separator().classes('flex-grow')  
                            ui.label('Parameters').classes('text-sm text-gray-500') 
                            with ui.row().classes('w-full items-center'):
                                with ui.button(f'Retrieve', on_click=lambda _, n=node_id: controller.motors[n].get_config()):
                                    ui.tooltip('Gets the current configs')
                                with ui.button(f'Apply', on_click=lambda _, n=node_id, p=[current_limit, config_mode, velocity, acceleration, deceleration, over_current, config_kp_gain, config_kd_gain, config_ki_gain]: controller.motors[n].set_config(p)):
                                    ui.tooltip('Applies the current configs')
                                with ui.button(f'Save', on_click=lambda _, n=node_id: controller.motors[n].save_configs(), color='red'):
                                    ui.tooltip('Saves the current configs to Motors MCU')
                            ui.separator().classes('flex-grow')  
                            ui.label('Controls').classes('text-sm text-gray-500') 
                            with ui.row().classes('w-full items-center'):
                                with ui.button(f'Home', on_click=lambda _, n=node_id: controller.motors[n].set_home()):
                                    ui.tooltip('Set current position as home position')
                                with ui.button(f'Normal', on_click=lambda _, n=node_id: controller.motors[n].set_speed_mode([3, 4.8, 4.8]) , color='green'):
                                    ui.tooltip('Robot arm speed in normal mode')
                                with ui.button(f'Fast', on_click=lambda _, n=node_id: controller.motors[n].set_speed_mode([7, 11, 11]) , color='blue'):
                                    ui.tooltip('Robot arm speed in fast mode')
                                with ui.button(f'Sport', on_click=lambda _, n=node_id: controller.motors[n].set_speed_mode([15, 24, 24]) , color='blue'):
                                    ui.tooltip('Robot arm speed in sport mode')
                                with ui.button(f'Crazzy', on_click=lambda _, n=node_id: controller.motors[n].set_speed_mode([30, 50, 50]), color='red'):
                                    ui.tooltip('Robot arm speed in crazzy mode')
                            
                            ui.separator().classes('flex-grow')
                            ui.label('Calibration & Homing').classes('text-sm text-gray-500')
                            with ui.row().classes('w-full items-center'):
                                with ui.button(f'Start Calibration', on_click=lambda _, n=node_id: run_calibration_for_motor(n), color='red'):
                                    ui.tooltip(f'Force physical calibration sequence for Motor {node_id}')

    # === Menu 3: Camera Vision (极限压力测试模式说明 & 实时温度监控) ===
    with ui.tab_panel(menu3):
        with ui.card().classes('w-full items-center'):
            ui.label('D415 多通道极限压力测试台').classes('text-h4 text-red-600')
            
            ui.html('''
                <div style="background-color: #fff1f0; border-left: 4px solid #ff4d4f; padding: 12px 20px; margin-bottom: 20px; width: 100%; border-radius: 4px; box-sizing: border-box;">
                    <h3 style="margin: 0 0 10px 0; font-size: 1.1rem; color: #cf1322;">🔥 当前系统处于极限压力测试模式：</h3>
                    <ul style="margin: 0; padding-left: 20px; color: #333; font-size: 0.95rem; line-height: 1.6;">
                        <li><b>底层硬件采集：</b>彻底关闭自适应保护，强制所有通道 (彩色/深度/左红外/右红外) 运行在 <b>640x480 @ 60 FPS</b>。无降级回退，带宽不足直接报错。</li>
                        <li><b>前端网页推流：</b>后端四通道合并单图，最高 <b>60 FPS</b> 极速推流。(测试局域网和前端渲染极限)。</li>
                        <li><b>⚠️ 警告：</b>多台相机同时启用该模式极易导致 USB 控制器崩溃或总线严重堵塞丢帧，请时刻注意下方实时温度监控，过热时请立刻停止。</li>
                    </ul>
                </div>
            ''')
            
            ui.html('''
                <style>
                    .grid-container { display: flex; flex-wrap: wrap; gap: 2%; justify-content: center; width: 100%; margin-bottom: 20px;}
                    .cam-row { display: flex; flex-direction: column; align-items: center; width: 32%; min-width: 320px; background-color: #f4f4f4; border-radius: 8px; padding: 12px; border: 1px solid #ddd; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
                    .cam-img { width: 100%; aspect-ratio: 4/3; object-fit: contain; background-color: #000; border-radius: 4px; margin-bottom: 8px;}
                    .cam-title { width: 100%; text-align: center; color: #333; font-size: 1.1rem; margin-bottom: 4px; font-weight: bold; }
                    .cam-desc { width: 100%; text-align: center; color: #d9363e; font-size: 0.85rem; margin-bottom: 4px; font-weight: bold;}
                    .cam-temp { width: 100%; text-align: center; color: #e65100; font-size: 0.95rem; margin-bottom: 8px; font-weight: bold;}
                </style>
                
                <div class="grid-container">
                    <div class="cam-row">
                        <div class="cam-title">📷 相机 0 (全通道全开)</div>
                        <div class="cam-desc">硬件设定: 640x480 | 推流上限: 60 FPS</div>
                        <div id="cam_0_temp" class="cam-temp">实时温度: 获取中...</div>
                        <img id="cam_0_feed" class="cam-img" src="" />
                    </div>
                    
                    <div class="cam-row">
                        <div class="cam-title">📷 相机 1 (全通道全开)</div>
                        <div class="cam-desc">硬件设定: 640x480 | 推流上限: 60 FPS</div>
                        <div id="cam_1_temp" class="cam-temp">实时温度: 获取中...</div>
                        <img id="cam_1_feed" class="cam-img" src="" />
                    </div>
                    
                    <div class="cam-row">
                        <div class="cam-title">📷 相机 2 (全通道全开)</div>
                        <div class="cam-desc">硬件设定: 640x480 | 推流上限: 60 FPS</div>
                        <div id="cam_2_temp" class="cam-temp">实时温度: 获取中...</div>
                        <img id="cam_2_feed" class="cam-img" src="" />
                    </div>
                </div>
            ''')
            
            with ui.row().classes('mt-2 gap-4'):
                ui.button('Start Cameras', on_click=start_camera, color='green').classes('w-40')
                ui.button('Stop Cameras', on_click=stop_camera, color='red').classes('w-40')

    # === Menu 4: Live Charts (新增实时曲线图表) ===
    with ui.tab_panel(menu4):
        with ui.card().classes('w-full'):
            ui.label('Motor Telemetry / 关节实时曲线监控').classes('text-h4')
            
            ui.html('''
                <div style="color: #666; font-size: 0.95rem; margin-bottom: 15px;">
                    💡 提示：本页面展示所有 7 个关节的实时数据变化。支持鼠标悬浮查看具体数值、框选区域放大，以及点击底部图例单独隐藏/显示特定电机的曲线。
                </div>
            ''')
            
            # 修复了 ui.echart 的拼写错误
            with ui.column().classes('w-full gap-8'):
                pos_chart = ui.echart(create_echart_options('Joint Position (位置)', 'Turns')).classes('w-full h-80')
                vel_chart = ui.echart(create_echart_options('Joint Velocity (速度)', 'Speed')).classes('w-full h-80')
                torq_chart = ui.echart(create_echart_options('Joint Torque (力/力矩)', 'Torque')).classes('w-full h-80')

# ================= 独立硬件轮询线程 =================
def hardware_polling_loop():
    while True:
        try:
            get_all_motors_status()
        except Exception as e:
            print(f"Polling error: {e}")
        time.sleep(0.5)

app.on_startup(lambda: threading.Thread(target=hardware_polling_loop, daemon=True).start())

# 定时器：更新电机状态表盘、相机实时温度、以及 实时曲线图表 数据绑定
ui.timer(0.5, update_status_display)

# 系统退出时注销事件
app.on_shutdown(controller.stop)
app.on_shutdown(stop_camera)

ui.run()