from core.paths import MOTORS_CONFIG, RECORDING_FILE, prepare_runtime

prepare_runtime()
from nicegui import ui, app
from core.motorcontroller import MotorController
from core.gui_control import GuiControl, Rejected, finite
from core.gui_journal import Journal, updated_at, clean_json
from core.paths import RUNTIME_DIR
import argparse
import copy
import fcntl
import sys
import signal
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
recording_file = RECORDING_FILE

# === 多相机 & 四通道(拼图)全局变量 ===
pipelines = []
camera_running = False
capture_threads = []
latest_frame_bytes = {}  
latest_camera_temps = {}

# === 实时数据曲线图表 全局变量 ===
MAX_HISTORY = 60  
chart_timestamps = deque(maxlen=MAX_HISTORY)
chart_data = {
    'position': {i: deque(maxlen=MAX_HISTORY) for i in range(1, 8)},
    'velocity': {i: deque(maxlen=MAX_HISTORY) for i in range(1, 8)},
    'torque':   {i: deque(maxlen=MAX_HISTORY) for i in range(1, 8)}
}

def create_echart_options(title, y_axis_name):
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
    global camera_running, latest_frame_bytes, latest_camera_temps
    W_MAIN, H_MAIN = 640, 480
    depth_sensor = None
    try:
        active_dev = pipeline.get_active_profile().get_device()
        sensors = active_dev.query_sensors()
        depth_sensor = next((s for s in sensors if s.is_depth_sensor()), None)
    except Exception as e:
        print(f"获取相机 {cam_idx} 传感器失败: {e}")
    
    while camera_running and pipeline:
        try:
            if depth_sensor and depth_sensor.supports(rs.option.asic_temperature):
                latest_camera_temps[cam_idx] = depth_sensor.get_option(rs.option.asic_temperature)

            frames = pipeline.wait_for_frames(timeout_ms=2000)
            aligned_frames = align.process(frames) if align else frames
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            ir_left_frame = frames.get_infrared_frame(1) 
            ir_right_frame = frames.get_infrared_frame(2)

            color_image = np.zeros((H_MAIN, W_MAIN, 3), dtype=np.uint8)
            depth_colormap = np.zeros((H_MAIN, W_MAIN, 3), dtype=np.uint8)
            ir_left_resized = np.zeros((H_MAIN, W_MAIN, 3), dtype=np.uint8)
            ir_right_resized = np.zeros((H_MAIN, W_MAIN, 3), dtype=np.uint8)

            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            if ir_left_frame:
                ir_left_raw = np.asanyarray(ir_left_frame.get_data())
                ir_left_bgr = cv2.cvtColor(ir_left_raw, cv2.COLOR_GRAY2BGR)
                ir_left_resized = cv2.resize(ir_left_bgr, (W_MAIN, H_MAIN))
            if ir_right_frame:
                ir_right_raw = np.asanyarray(ir_right_frame.get_data())
                ir_right_bgr = cv2.cvtColor(ir_right_raw, cv2.COLOR_GRAY2BGR)
                ir_right_resized = cv2.resize(ir_right_bgr, (W_MAIN, H_MAIN))

            top_row = np.hstack((color_image, depth_colormap))
            bottom_row = np.hstack((ir_left_resized, ir_right_resized))
            grid = np.vstack((top_row, bottom_row))
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(grid, 'Color', (20, 40), font, 1.2, (255, 255, 255), 2)
            cv2.putText(grid, 'Depth', (W_MAIN + 20, 40), font, 1.2, (255, 255, 255), 2)
            cv2.putText(grid, 'IR Left', (20, H_MAIN + 40), font, 1.2, (255, 255, 255), 2)
            cv2.putText(grid, 'IR Right', (W_MAIN + 20, H_MAIN + 40), font, 1.2, (255, 255, 255), 2)

            grid_small = cv2.resize(grid, (0, 0), fx=0.5, fy=0.5)

            ret, buffer = cv2.imencode('.jpg', grid_small, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
            if ret:
                latest_frame_bytes[cam_idx] = buffer.tobytes()
        except Exception as e:
            latest_frame_bytes.pop(cam_idx, None)
            latest_camera_temps.pop(cam_idx, None)
            message_queue.put(f'相机 {cam_idx} 帧获取失败：{e}')
            time.sleep(1)  

async def generate_frames(cam_idx):
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
        
        for i, dev in enumerate(devices):
            if i >= MAX_CAMERAS: 
                break 
                
            sn = dev.get_info(rs.camera_info.serial_number)
            pipeline = rs.pipeline(ctx)
            config = rs.config()
            config.enable_device(sn)
            
            W_MAIN, H_MAIN = 640, 480 
            FPS = 30
            config.enable_stream(rs.stream.color, W_MAIN, H_MAIN, rs.format.bgr8, FPS)
            config.enable_stream(rs.stream.depth, W_MAIN, H_MAIN, rs.format.z16, FPS)
            config.enable_stream(rs.stream.infrared, 1, W_MAIN, H_MAIN, rs.format.y8, FPS)
            config.enable_stream(rs.stream.infrared, 2, W_MAIN, H_MAIN, rs.format.y8, FPS)
            
            try:
                if config.can_resolve(pipeline):
                    pipeline.start(config)
                    pipelines.append(pipeline)
                    ui.notify(f"Camera {i+1} started (30 FPS)", color='green')
                else:
                    raise RuntimeError("带宽或硬件不支持 30 FPS，拒绝启动")
            except Exception as e:
                ui.notify(f"Camera {i+1} start failed: {e}", type='negative')
                continue
            
            t = threading.Thread(target=camera_capture_loop, args=(i, pipeline, rs.align(rs.stream.color)), daemon=True)
            t.start()
            capture_threads.append(t)
            ui.run_javascript(f'try {{ document.getElementById("cam_{i}_feed").src = "/video_feed/{i}?t=" + new Date().getTime(); }} catch(e) {{}}')
        camera_running = bool(pipelines)
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


# First-stage control surface: every motion enters GuiControl.
parser = argparse.ArgumentParser(description='DummyX 机械臂控制台')
parser.add_argument('--host', default='127.0.0.1')
parser.add_argument('--port', type=int, default=8080)
parser.add_argument('--virtual', action='store_true', help='内存 CAN，不连接真实机械臂')
arguments = sys.argv[1:]
if arguments and arguments[0] == 'gui':
    arguments = arguments[1:]
options = parser.parse_args(arguments)

# Acquire before opening CAN; OS releases the lock if the process exits.
instance_lock = (RUNTIME_DIR / ('gui-virtual.lock' if options.virtual else 'gui-can0.lock')).open('a')
try:
    fcntl.flock(instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    raise SystemExit('已有 GUI 控制进程占用此总线，请先关闭旧进程')

with MOTORS_CONFIG.open(encoding='utf-8') as source:
    motor_config = yaml.safe_load(source)
controller = MotorController(interface='virtual' if options.virtual else 'socketcan',
                             channel='gui-preview' if options.virtual else 'can0')
for node in motor_config['nodes']:
    if controller.is_initialized():
        motor = controller.add_motor(node['id'], node['reduction'])
        motor.send_timeout = 0.2
journal = Journal(RUNTIME_DIR / ('logs-virtual' if options.virtual else 'logs'))
control = GuiControl(controller, motor_config['nodes'],
                     homing_defaults=motor_config.get('homing_defaults'),
                     commissioning_defaults=motor_config.get('commissioning_defaults'),
                     manual_defaults=motor_config.get('manual_defaults'), journal=journal)
control.events.extend(journal.recent())
control.log('GUI 控制台启动')
message_queue = Queue(maxsize=500)
poll_stop = threading.Event()
poll_thread = None
motion_widgets = []
group_motion_widgets = []
commissioning_widgets = []
joint_target_inputs = {}
camera_temp_labels = []
recorded_data = []
is_recording = False
recording_start = None
recording_error = ''
history = deque(maxlen=120)
exit_requested = False
exit_returning = False
exit_authorized = False
exit_started_at = None
exit_stage = '待确认'
previous_signal_handlers = {}
node_alarm_states = {}
last_alarm_view = None


async def export_diagnostics():
    # Both operations are bounded local I/O. Keeping them in this event loop avoids
    # orphaned executor futures when shutdown has already been requested.
    journal.flush()
    events = journal.recent(100000, journal.session)
    payload = clean_json({'exported_at': datetime.now().astimezone().isoformat(),
                          'session': journal.session, 'log_error': journal.error,
                          'config': motor_config, 'bus': controller.get_bus_health(),
                          'status': controller.get_all_motor_status(), 'events': events,
                          'scope': '本次运行中仍保留在滚动日志内的记录'})
    ui.download(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False).encode(),
                f'dummyx-debug-{datetime.now():%Y%m%d-%H%M%S}.json')


async def load_saved_events():
    records = await asyncio.to_thread(journal.recent, 200)
    with control.lock:
        # Do not replace live records while the background read is running.
        by_key = {(e['time'], e['message']): e for e in records}
        by_key.update({(e['time'], e['message']): e for e in control.events})
        control.events.clear()
        control.events.extend(sorted(by_key.values(), key=lambda e: e['time'])[-200:])


def request_exit(source='界面'):
    global exit_requested, exit_stage
    if exit_authorized or exit_returning:
        return
    if not exit_requested:
        exit_requested = True
        exit_stage = '待确认'
        if control.active:
            control.cancel('收到退出请求，取消当前任务')
        control.log(f'{source}请求退出：请在界面选择归位后退出、直接退出或取消退出')
        print('退出待确认：请在 GUI 选择归位后退出、直接退出或取消退出。', flush=True)


def cancel_exit():
    global exit_requested, exit_stage
    if not exit_returning:
        exit_requested = False
        exit_stage = '已取消'


async def finish_exit(return_to_pose):
    global exit_requested, exit_returning, exit_authorized, exit_started_at, exit_stage
    if exit_returning or exit_authorized:
        return
    exit_returning = True
    try:
        if return_to_pose:
            exit_stage = '归位中'
            exit_started_at = time.monotonic()
            targets = {int(n): finite(v, f'J{n} 工作结束角度')
                       for n, v in motor_config['end_pose'].items()}
            if set(targets) != set(control.nodes):
                raise Rejected('工作结束姿态必须包含全部关节')
            control.move(targets, speed_scale(), '工作结束归位',
                         require_target_reached=False, position_tolerance=1.0)
            worker = control.worker
            while worker.is_alive():
                await asyncio.sleep(.1)
            if control.latched or control.cancel_event.is_set():
                raise Rejected('工作结束归位未完成，保留程序运行；请检查事件记录')
            exit_stage = '失能确认中'
            disable_errors = []

            def disable_after_return():
                try:
                    control.disable_and_confirm()
                except Exception as error:
                    disable_errors.append(error)

            disable_worker = threading.Thread(target=disable_after_return, daemon=True,
                                              name='gui-exit-disable')
            disable_worker.start()
            while disable_worker.is_alive():
                await asyncio.sleep(.1)
            if disable_errors:
                raise disable_errors[0]
            exit_stage = '正在终止程序'
        else:
            exit_stage = '直接退出'
            control.cancel('用户确认直接退出；未执行工作结束归位')
            worker = control.worker
            if worker:
                while worker.is_alive():
                    await asyncio.sleep(.1)
        exit_authorized = True
        app.shutdown()
    except Exception as error:
        exit_stage = '失败，程序保持运行'
        control.log(f'退出未完成：{error}')
        ui.notify(str(error), type='negative')
    finally:
        exit_returning = False


def receive_exit_signal(signum, frame):
    request_exit(signal.Signals(signum).name)


async def return_and_exit():
    await finish_exit(True)


async def exit_without_return():
    await finish_exit(False)


def invoke(action):
    try:
        action()
    except Exception as error:
        control.log(f'拒绝操作：{error}')
        ui.notify(str(error), type='negative')


def confirm(title, details, action):
    with ui.dialog() as dialog, ui.card().classes('w-96'):
        ui.label(title).classes('text-lg font-bold')
        ui.label(details)
        with ui.row():
            ui.button('取消', on_click=dialog.close).props('flat')
            def accept():
                dialog.close()
                invoke(action)
            ui.button('确认执行', on_click=accept)
    dialog.open()


def request_move():
    node_id = int(joint_select.value)
    target = finite(position_input.value, '目标角度')
    confirm('执行定位', f'J{node_id} → {target}°；沿途碰撞未自动检查。',
            lambda: control.move({node_id: target}, speed_scale(), f'J{node_id} 定位'))


def speed_scale():
    return finite(manual_speed.value, '速度倍率') / 100


def request_jog(direction):
    step = finite(jog_step.value, '点动步距')
    control.jog(int(joint_select.value), direction * step, speed_scale())


def fill_group_from_current():
    statuses = control.check_nodes(control.nodes, enabled=True, limits=True)
    for node_id, field in joint_target_inputs.items():
        field.value = round(statuses[node_id]['position'], 3)
        field.update()
    control.log('已将全部关节的新鲜当前位置填入整组目标；尚未执行运动')


def request_group_move():
    targets = {node_id: finite(field.value, f'J{node_id} 目标角度')
               for node_id, field in joint_target_inputs.items()}
    control.validate_targets(targets)
    details = '，'.join(f'J{node_id}={target:g}°' for node_id, target in targets.items())
    confirm('执行七关节整组定位', details + '。仅检查关节软限位，不检查空间路径碰撞。',
            lambda: control.move(targets, speed_scale(), '七关节整组定位'))


def request_named_pose(name, targets):
    control.validate_targets(targets)
    details = '，'.join(f'J{node_id}={target:g}°' for node_id, target in targets.items())
    confirm(f'移动到{name}', details + '。确认机械臂到目标之间的空间路径畅通。',
            lambda: control.move(targets, speed_scale(), name))


def request_loop():
    node_id = int(joint_select.value)
    target = finite(position_input.value, '往返角度')
    count = finite(loop_count.value, '循环次数')
    if not count.is_integer() or not 1 <= count <= 100:
        raise Rejected('循环次数必须为 1–100 的整数')
    steps = [({node_id: value}, 0.25)
             for _ in range(int(count)) for value in (-target, target)]
    confirm('执行往返', f'J{node_id} 在 ±{target}° 间运动 {int(count)} 次。',
            lambda: control.sequence(steps, '往返循环', speed_scale()))


def request_commissioning_move(delta):
    control.commissioning_move(int(commissioning_joint.value), delta)


def capture_limit(field):
    node_id = int(commissioning_joint.value)
    reason = control.commissioning_reason(node_id)
    if reason:
        raise Rejected(reason)
    field.value = round(control.check_nodes([node_id], enabled=True)[node_id]['position'], 3)
    field.update()


def request_save_limits():
    node_id = int(commissioning_joint.value)
    minimum = finite(limit_min_input.value, '最小软限位')
    maximum = finite(limit_max_input.value, '最大软限位')
    control.validate_commissioning_limits(node_id, minimum, maximum)

    def save():
        with control.lock:
            checked = control.validate_commissioning_limits(node_id, minimum, maximum)
            candidate = copy.deepcopy(motor_config)
            target = next((node for node in candidate['nodes'] if node['id'] == node_id), None)
            if target is None:
                raise Rejected(f'配置中找不到 J{node_id}')
            target['limits'] = checked
            temporary = MOTORS_CONFIG.with_suffix('.yaml.tmp')
            temporary.write_text(yaml.safe_dump(candidate, allow_unicode=True, sort_keys=False),
                                 encoding='utf-8')
            temporary.replace(MOTORS_CONFIG)
            live = next(node for node in motor_config['nodes'] if node['id'] == node_id)
            live['limits'] = checked
            control.nodes[node_id]['limits'] = checked
            control.log(f"J{node_id} 软限位已保存并立即生效：[{minimum:g}, {maximum:g}]°")

    confirm(f'核实 J{node_id} 软限位',
            f'保存 [{minimum:g}, {maximum:g}]° 并标记 verified=true。确认已留出机械止挡和碰撞安全余量。',
            save)


def begin_recording():
    global is_recording, recorded_data, recording_start, recording_error
    if is_recording:
        raise Rejected('正在录制')
    control.check_nodes(control.nodes)
    recorded_data = []
    recording_error = ''
    recording_start = time.monotonic()
    is_recording = True
    control.log('开始录制新鲜位置反馈（2 Hz，最多 10000 点）')


def finish_recording():
    global is_recording
    is_recording = False
    if not recorded_data:
        raise Rejected('没有可保存的数据')
    temporary = recording_file.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(recorded_data, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(recording_file)
    control.log(f'已保存 {len(recorded_data)} 个点到 {recording_file.name}')


def replay():
    if is_recording:
        raise Rejected('请先结束录制')
    if recording_file.stat().st_size > 10_000_000:
        raise Rejected('轨迹文件超过 10 MB')
    data = json.loads(recording_file.read_text(encoding='utf-8'))
    if not isinstance(data, list) or not 1 <= len(data) <= 10000:
        raise Rejected('轨迹必须包含 1–10000 条记录')
    steps = []
    for record in data:
        positions = record['positions']
        targets = {}
        for entry in positions:
            node_id = entry['node_id']
            if isinstance(node_id, bool) or not isinstance(node_id, int) or node_id in targets:
                raise Rejected('轨迹节点 ID 非法或重复')
            targets[node_id] = entry['position']
        steps.append((targets, 0.5))
    # Position teaching replay is point-to-point, never a timed trajectory interpolator.
    control.sequence(steps)


def poll():
    cycle = 0

    def request_feedback(node_id, spacing=0.03):
        motor = controller.motors[node_id]
        motor.reference_status()
        poll_stop.wait(spacing)
        if control.active == '自动碰撞调零' and control.active_node == node_id:
            motor.reference_motion_feedback(inter_request_delay=spacing)
        else:
            motor.reference_position_feedback()

    while not poll_stop.is_set():
        cycle += 1
        try:
            if not controller.is_initialized():
                control.communication_error = 'CAN 初始化失败，请检查接口后重启'
            else:
                node_ids = list(controller.motors)
                offset = (cycle - 1) % len(node_ids) if node_ids else 0
                ordered_nodes = node_ids[offset:] + node_ids[:offset]
                for node_id in ordered_nodes:
                    if poll_stop.is_set():
                        return
                    request_feedback(node_id)
                    poll_stop.wait(0.02)
                # Preserve the UI current display without polling all currents at
                # high rate: sample one inactive node per cycle.
                if node_ids:
                    controller.motors[node_ids[(cycle - 1) % len(node_ids)]].reference_current_feedback()
                # One bounded retry for nodes whose status or position is already
                # aging. This repairs an isolated dropped query without flooding CAN.
                aging = [status['node_id'] for status in controller.get_all_motor_status()
                         if max(status['status_age'], status['position_age']) > 1.0]
                for node_id in aging:
                    if poll_stop.is_set():
                        return
                    request_feedback(node_id, spacing=0.05)
                with control.lock:
                    control.communication_error = ''
                if control.active and control.active not in {'故障复位', '自动碰撞调零'}:
                    try:
                        # Active command workers resend idempotent commands while
                        # position remains fresh. Do not cancel them merely because
                        # one independent status reply aged out.
                        control.active_statuses(control.nodes)
                    except Rejected as error:
                        control.cancel(f'通信 / 故障监测：{error}')
        except Exception as error:
            with control.lock:
                control.communication_error = str(error)
            if control.active:
                control.cancel(f'通信异常：{error}')
        poll_stop.wait(0.2)


def startup():
    global poll_thread
    controller.start()
    poll_thread = threading.Thread(target=poll, daemon=True, name='gui-poll')
    poll_thread.start()
    # Installed after Uvicorn's handlers, before accepting normal UI work.
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_signal_handlers[sig] = signal.getsignal(sig)
        asyncio.get_running_loop().add_signal_handler(sig, receive_exit_signal, sig, None)


def disconnected():
    global is_recording, recording_error
    if control.active:
        control.cancel('浏览器连接已断开')
    if is_recording:
        is_recording = False
        recording_error = '浏览器断连，录制已暂停；重连后可保存'
    if not exit_authorized:
        request_exit('浏览器断连')


def shutdown():
    global camera_running, is_recording
    is_recording = False
    poll_stop.set()
    control.close()
    journal.close()
    if poll_thread:
        poll_thread.join(timeout=3)
    camera_running = False
    for pipeline in list(pipelines):
        try:
            pipeline.stop()
        except Exception:
            pass
    controller.stop()
    if controller.bus:
        controller.bus.shutdown()
    instance_lock.close()
    for sig, handler in previous_signal_handlers.items():
        signal.signal(sig, handler)


def refresh():
    global is_recording, recording_error, last_alarm_view
    for _ in range(20):
        if message_queue.empty():
            break
        control.log(message_queue.get_nowait())
    snapshots = {s['node_id']: s for s in controller.get_all_motor_status()}
    exit_panel.set_visibility(exit_requested)
    for button in exit_buttons:
        button.set_enabled(not exit_returning)
    rows = []
    online = 0
    for node_id in control.nodes:
        status = snapshots.get(node_id)
        fresh = status is not None and control.fresh(status)
        online += int(fresh)
        errors = ', '.join(k for k, value in status['errors'].items() if value) if status else ''
        health = '驱动器故障' if errors else ('正常' if fresh else '反馈过期')
        if node_alarm_states.get(node_id) != (health, errors):
            node_alarm_states[node_id] = (health, errors)
            control.log(f'J{node_id} {health}' + (f'：{errors}' if errors else ''),
                        category=health, update_result=False)
        rows.append({
            'joint': f'J{node_id}',
            'connection': '在线' if fresh else '离线 / 数据过期',
            'enabled': ('已使能' if status['enabled'] else '已失能') if fresh else '未知',
            'position': f"{status['position']:.3f}" if fresh else '—',
            'target': (f"{control.last_targets[node_id]:.3f}"
                       if control.last_targets[node_id] is not None else '—'),
            'motion': (('到位' if status['target_reached'] else '运动中 / 未到位')
                       if fresh and control.last_targets[node_id] is not None else '—'),
            'current': f"{status['current']:.3f}" if fresh else '—',
            'fault': errors or ('无' if fresh else '未知'),
            'homed': '已完成' if node_id in control.homed_nodes else '未完成',
            'status_updated': updated_at(status.get('status_received_at')) if status else '尚未收到',
            'position_updated': updated_at(status.get('position_received_at')) if status else '尚未收到',
        })
    status_table.rows = rows
    status_table.update()
    reason = control.reason([int(joint_select.value)])
    state_label.set_text('软件停止锁定' if control.latched else
                         (control.active or ('待命' if not reason else '运动未就绪')))
    bus_health = controller.get_bus_health()
    connection_label.set_text(
        f"{'虚拟 CAN' if options.virtual else 'CAN0'} · {online}/{len(control.nodes)} 节点在线 · "
        f"CAN 错误帧 {bus_health['error_frames']}")
    permission_label.set_text(reason or '选定关节检查通过；运动仍需使能反馈')
    result_label.set_text(control.last_result)
    progress_label.set_text(control.progress)
    alarm_summary.set_text(('软件锁定 · ' if control.latched else '') +
                           ('正在重试' if control.active and '重发' in control.progress else
                            control.active or '待命') + ' · ' + control.last_result)
    journal_status.set_text(journal.error or '日志自动保存；重启后保留，最多约 30 MB 滚动存储')
    exit_rows = []
    for n, target in motor_config['end_pose'].items():
        n = int(n)
        s = snapshots.get(n)
        target_sent = control.last_targets.get(n) == target
        reached = bool(s and control.fresh(s) and target_sent
                       and exit_started_at is not None
                       and (s['position_received_at'] or 0) > exit_started_at
                       and (s['status_received_at'] or 0) > exit_started_at
                       and abs(s['position'] - target) <= 1)
        exit_rows.append({'joint': f'J{n}', 'target': target,
                          'position': s['position'] if s and s['position_age'] <= control.stale_seconds else '—',
                          'state': ('正在确认失能' if exit_stage == '失能确认中' else
                                    '退出失败 / 已锁定' if exit_stage == '失败，程序保持运行' else
                                    '未执行' if not exit_returning else
                                    '已到位' if reached else '等待反馈 / 归位中')})
    exit_progress.rows = exit_rows
    exit_progress.update()
    for widget in motion_widgets:
        widget.set_enabled(not bool(reason))
    group_reason = control.reason()
    for widget in group_motion_widgets:
        widget.set_enabled(not bool(group_reason))
    enable_button.set_enabled(not bool(control.reason()))
    homing_button.set_enabled(not bool(control.homing_reason()))
    commissioning_node = int(commissioning_joint.value)
    commissioning_reason = control.commissioning_reason(commissioning_node)
    for widget in commissioning_widgets:
        widget.set_enabled(not bool(commissioning_reason))
    commissioning_status.set_text(commissioning_reason or
                                  f'J{commissioning_node} 调试就绪；每次最多移动 {control.commissioning_step_max:g}°')
    selected_status = snapshots.get(commissioning_node)
    commissioning_position.set_text(
        f"当前位置：{selected_status['position']:.3f}°" if selected_status and control.fresh(selected_status)
        else '当前位置：无新鲜反馈')
    limit_rows = []
    for node in motor_config['nodes']:
        spec = node.get('limits') or {}
        limit_rows.append({'joint': f"J{node['id']}", 'min': spec.get('min_deg'),
                           'max': spec.get('max_deg'), 'verified': spec.get('verified') is True})
    limit_table.rows = limit_rows
    limit_table.update()
    with control.lock:
        records = list(reversed(control.events))
    event_log.rows = records
    event_log.update()
    selection = (alarm_filter.value, records[0]['time'] if records else None,
                 records[0]['message'] if records else None)
    if selection != last_alarm_view:
        last_alarm_view = selection
        detail_rows = []
        for event in records:
            if event.get('category') not in {'任务超时', '驱动器故障', '任务失败', '软件锁定', '正在重试', '反馈过期'}:
                continue
            if alarm_filter.value != '全部' and event.get('category') != alarm_filter.value:
                continue
            for s in event.get('details', []) or [{}]:
                n = s.get('node_id')
                detail_rows.append({'time': event['time'], 'category': event.get('category'),
                                   'joint': f'J{n}' if n else '—', 'stage': event.get('stage') or event.get('task') or '—',
                                   'target': s.get('target'), 'position': s.get('position'),
                                   'current': s.get('current'), 'updated': s.get('status_updated', '尚未收到'),
                                   'position_updated': s.get('position_updated', '尚未收到'),
                                   'current_updated': s.get('current_updated', '尚未收到'),
                                   'fault': ', '.join(k for k, v in s.get('errors', {}).items() if v) or '无',
                                   'message': event['message']})
        alarm_table.rows = detail_rows[:1400]
        alarm_table.update()
    for i, label in enumerate(camera_temp_labels):
        temp = latest_camera_temps.get(i)
        label.set_text(f'温度：{temp:.1f} °C' if temp is not None else '温度：未连接 / 无反馈')
    recording_label.set_text(f"{'录制中' if is_recording else '未录制'} · {len(recorded_data)} 点 " + recording_error)
    if is_recording:
        try:
            control.active_statuses(control.nodes)
            if len(recorded_data) >= 10000:
                raise Rejected('录制达到 10000 点上限，请保存')
            recorded_data.append({
                'timestamp': datetime.now().isoformat(timespec='milliseconds'),
                'elapsed': time.monotonic() - recording_start,
                'positions': [{'node_id': n, 'position': s['position']} for n, s in snapshots.items()],
            })
        except Rejected as error:
            is_recording = False
            recording_error = str(error)
            control.log(f'录制已停止：{error}；已有数据可保存')
    history.append((datetime.now().strftime('%H:%M:%S'),
                    {n: s['position'] if control.fresh(s) else None for n, s in snapshots.items()}))
    chart.options['xAxis']['data'] = [item[0] for item in history]
    for series, node_id in zip(chart.options['series'], control.nodes):
        series['data'] = [item[1].get(node_id) for item in history]
    chart.update()


ui.add_css('''
body { background: #f1f5f9; color: #172033; }
.q-card { border: 1px solid #dbe3ed; box-shadow: none; border-radius: 10px; }
.q-table th { font-weight: 700; background: #f8fafc; }
''')
with ui.header().classes('bg-slate-900 items-center justify-between'):
    with ui.column().classes('gap-0'):
        ui.label('DummyX · 机械臂控制台').classes('text-xl font-bold')
        connection_label = ui.label('正在检查通信').classes('text-sm')
    state_label = ui.label('运动未就绪').classes('text-lg')
    ui.button('取消当前任务', on_click=lambda: control.cancel(), color='orange')
    ui.button('全部失能', on_click=control.disable, color='red')
    ui.button('结束工作 / 退出程序', on_click=lambda: request_exit())
with ui.column().classes('w-full max-w-screen-xl mx-auto p-4 gap-3'):
    with ui.card().classes('w-full bg-amber-50') as exit_panel:
        ui.label('退出前：是否移动到工作结束姿态？').classes('text-lg font-bold')
        ui.label(' · '.join(f'J{n} {target:g}°'
                            for n, target in motor_config['end_pose'].items()))
        ui.label('归位按当前速度倍率执行；归位成功后将全部失能，确认新鲜失能反馈后终止 GUI 和当前终端命令。')
        ui.label('失能可能引起机械臂下坠；仅在确认该结束姿态无需电机承力时使用。直接退出不会主动失能。').classes('text-red-800')
        exit_progress = ui.table(columns=[{'name': k, 'field': k, 'label': label}
                                         for k, label in [('joint', '关节'), ('target', '结束目标 (°)'),
                                                          ('position', '当前位置 (°)'), ('state', '归位进度')]],
                                 rows=[]).classes('w-full')
        with ui.row():
            exit_buttons = [
                ui.button('归位、确认失能后退出', on_click=return_and_exit),
                ui.button('不归位，直接退出', on_click=exit_without_return, color='orange'),
                ui.button('取消退出', on_click=cancel_exit),
            ]
    exit_panel.set_visibility(False)
    ui.label('取消任务只阻止后续指令；全部失能可能引起重力下坠。硬件急停状态：未接入。').classes('text-amber-900')
    with ui.card().classes('w-full'):
        permission_label = ui.label('检查中')
        result_label = ui.label('等待操作').classes('text-sm')
        progress_label = ui.label('')
        with ui.row():
            enable_button = ui.button('使能全部关节', on_click=lambda: confirm(
                '使能全部关节', '确认机械臂周围无人、当前姿态与驱动器目标一致，并已核实软限位。', control.enable))
            ui.button('解除软件锁定', on_click=lambda: confirm(
                '解除软件锁定', '此操作不恢复旧任务、不清除驱动器故障。确认设备已停止且状态正常。', control.unlock)).props('outline')
            ui.button('复位驱动器故障', on_click=lambda: confirm(
                '复位驱动器故障', '仅在全部关节失能时执行；请先排除故障原因。', control.reset_errors)).props('outline')
    with ui.tabs().classes('w-full') as tabs:
        overview = ui.tab('运行控制')
        teaching = ui.tab('示教记录')
        settings = ui.tab('配置与维护')
        vision = ui.tab('相机')
        telemetry = ui.tab('位置曲线')
        events = ui.tab('报警与日志')
    with ui.tab_panels(tabs, value=overview).classes('w-full bg-transparent'):
        with ui.tab_panel(overview):
            with ui.card().classes('w-full'):
                ui.label('关节反馈').classes('text-lg font-bold')
                columns = [('joint', '关节'), ('connection', '通信'), ('enabled', '使能反馈'),
                           ('homed', '本次调零'), ('position', '角度 (°)'), ('current', '电流 (协议值)'), ('fault', '故障'),
                           ('target', '最近目标 (°)'), ('motion', '运动状态'),
                           ('status_updated', '状态更新时间'), ('position_updated', '位置更新时间')]
                status_table = ui.table(columns=[{'name': k, 'label': label, 'field': k}
                                                for k, label in columns], rows=[], row_key='joint').classes('w-full')
            with ui.card().classes('w-full mt-3'):
                ui.label('单关节手动控制').classes('text-lg font-bold')
                ui.label('点动为一次一指令；目标受软限位保护。按钮按下后不形成浏览器侧连续运动。').classes('text-sm')
                with ui.row().classes('items-end'):
                    joint_select = ui.select({n: f'J{n}' for n in control.nodes}, value=next(iter(control.nodes)),
                                             label='关节').classes('w-28')
                    position_input = ui.number('绝对目标 / 往返幅度 (°)', value=0, format='%.3f').classes('w-64')
                    loop_count = ui.number('循环次数', value=1, min=1, max=100, step=1).classes('w-28')
                    jog_step = ui.select({.1: '0.1°', .5: '0.5°', 1: '1°', 2: '2°'},
                                         value=.5, label='点动步距').classes('w-32')
                    manual_speed = ui.select({10: '10%', 25: '25%', 50: '50%', 100: '100%'},
                                             value=25, label='速度倍率').classes('w-32')
                with ui.row():
                    motion_widgets.append(ui.button('− 点动', on_click=lambda: invoke(
                        lambda: request_jog(-1))).props('outline'))
                    motion_widgets.append(ui.button('+ 点动', on_click=lambda: invoke(
                        lambda: request_jog(1))).props('outline'))
                    motion_widgets.append(ui.button('移动到目标', on_click=lambda: invoke(request_move)))
                    motion_widgets.append(ui.button('往返循环', on_click=lambda: invoke(request_loop)))
            with ui.card().classes('w-full mt-3'):
                ui.label('七关节整组姿态').classes('text-lg font-bold')
                ui.label('发送前一次性预检全部关节，再依次快速下发；关节空间移动不代表末端直线运动，也不自动检查自碰撞。').classes('text-sm text-amber-900')
                with ui.row().classes('items-end'):
                    for node_id in control.nodes:
                        limits = control.nodes[node_id]['limits']
                        initial = control.safe_pose.get(node_id)
                        joint_target_inputs[node_id] = ui.number(
                            f"J{node_id} [{limits['min_deg']:g}, {limits['max_deg']:g}]°",
                            value=initial, format='%.3f').classes('w-40')
                with ui.row():
                    group_motion_widgets.append(ui.button(
                        '填入当前位置', on_click=lambda: invoke(fill_group_from_current)).props('outline'))
                    group_motion_widgets.append(ui.button(
                        '执行整组目标', on_click=lambda: invoke(request_group_move), color='primary'))
                    group_motion_widgets.append(ui.button(
                        '安全姿态', on_click=lambda: invoke(lambda: request_named_pose(
                            '安全姿态', control.configured_safe_pose()))).props('outline'))
                    group_motion_widgets.append(ui.button(
                        '软限位中位姿态', on_click=lambda: invoke(lambda: request_named_pose(
                            '软限位中位姿态', control.midpoint_pose()))).props('outline'))
        with ui.tab_panel(teaching):
            ui.label('位置记录与逐点回放').classes('text-lg font-bold')
            ui.label('每 0.5 秒记录位置；回放逐点等待到位，非原速度轨迹复现。停止录制不停止机械臂。')
            recording_label = ui.label('未录制')
            with ui.row():
                ui.button('开始录制', on_click=lambda: invoke(begin_recording))
                ui.button('结束并保存录制', on_click=lambda: invoke(finish_recording))
                motion_widgets.append(ui.button('回放已保存记录', on_click=lambda: confirm(
                    '回放位置记录', '将预检全部轨迹点，覆盖文件中的关节。确认当前位置与首点之间的路径畅通。', replay)))
        with ui.tab_panel(settings):
            ui.label('自动碰撞调零').classes('text-lg font-bold')
            ui.label('调零不依赖软限位：按既有方向以低电流撞击机械限位、写入零点并回退。J5 会先调零避让，J7 使用较低速度。')
            ui.label('这是软件控制的维护运动，不是安全功能。执行前确认工作区无人、硬件急停可用，并全程监护。').classes('text-red-800')
            homing_button = ui.button('开始全部关节自动调零', color='orange', on_click=lambda: confirm(
                '危险操作：自动碰撞调零',
                '机械臂将依次主动运动并撞击物理限位。确认人员已撤离、机构可自由运动、急停在手边。',
                control.home_all))
            ui.separator()
            ui.label('调零后软限位调试').classes('text-lg font-bold')
            ui.label('仅在本次启动已完成全部调零后可用。一次只动一个关节，每次最多 2°，驱动器临时使用低速参数。')
            ui.label('临时包络只依据寻零方向和搜索行程，不能识别自碰撞或环境障碍；接近机械端点时请缩小步距并预留余量。').classes('text-amber-900')
            commissioning_status = ui.label('等待调零')
            commissioning_position = ui.label('当前位置：—')
            commissioning_joint = ui.select(
                {n: f'J{n}' for n in control.nodes}, value=next(iter(control.nodes)),
                label='调试关节').classes('w-32')
            with ui.row():
                for delta in (-2, -.5, .5, 2):
                    commissioning_widgets.append(ui.button(
                        f'{delta:+g}°', on_click=lambda _, step=delta: invoke(
                            lambda: request_commissioning_move(step))))
            with ui.row().classes('items-end'):
                limit_min_input = ui.number('最小软限位 (°)', value=None, format='%.3f').classes('w-48')
                ui.button('采集当前位置为最小值', on_click=lambda: invoke(
                    lambda: capture_limit(limit_min_input))).props('outline')
                limit_max_input = ui.number('最大软限位 (°)', value=None, format='%.3f').classes('w-48')
                ui.button('采集当前位置为最大值', on_click=lambda: invoke(
                    lambda: capture_limit(limit_max_input))).props('outline')
            commissioning_widgets.append(ui.button(
                '核实并保存该关节软限位', color='positive',
                on_click=lambda: invoke(request_save_limits)))
            ui.separator()
            ui.label('软限位配置').classes('text-lg font-bold')
            ui.label('这里填写的是调零后坐标系的安全工作范围。未知时仍可执行上述自动调零，但禁止普通使能、点动和轨迹运行。')
            limit_rows = []
            for n in motor_config['nodes']:
                spec = n.get('limits') or {}
                limit_rows.append({'joint': f"J{n['id']}", 'min': spec.get('min_deg'),
                                   'max': spec.get('max_deg'), 'verified': spec.get('verified') is True})
            limit_table = ui.table(columns=[{'name': k, 'label': label, 'field': k} for k, label in
                                             [('joint', '关节'), ('min', '最小角度 (°)'), ('max', '最大角度 (°)'),
                                              ('verified', '已人工核实')]], rows=limit_rows)
            ui.separator()
            ui.label('其他维护功能待验证').classes('text-lg font-bold')
            ui.label('底层标定、手动设置零点、阻尼模式及通用驱动器参数写入暂不可用。')
            ui.label('Ready / Max. Range / 夹爪预置角度待重新核实，当前请使用单关节绝对定位。')
        with ui.tab_panel(vision):
            ui.label('RealSense 多相机预览 · 640×480 / 30 FPS').classes('text-lg font-bold')
            with ui.row():
                ui.button('启动相机', on_click=start_camera)
                ui.button('停止相机', on_click=stop_camera)
            with ui.row().classes('w-full'):
                for i in range(3):
                    with ui.column().classes('w-1/3 min-w-60'):
                        ui.label(f'相机 {i}')
                        camera_temp_labels.append(ui.label('温度：未连接'))
                        ui.html(f'<img id="cam_{i}_feed" style="width:100%" src="" />')
        with ui.tab_panel(telemetry):
            chart = ui.echart({
                'tooltip': {'trigger': 'axis'},
                'legend': {'data': [f'J{n}' for n in control.nodes]},
                'xAxis': {'type': 'category', 'data': []},
                'yAxis': {'type': 'value', 'name': '关节角度 (°)'},
                'series': [{'name': f'J{n}', 'type': 'line', 'showSymbol': False,
                            'connectNulls': False, 'data': []} for n in control.nodes],
            }).classes('w-full h-96')
            ui.label('最近 60 秒；反馈过期显示断点，不补零。')
        with ui.tab_panel(events):
            alarm_summary = ui.label('待命').classes('text-lg font-bold')
            journal_status = ui.label('日志自动保存')
            with ui.row():
                ui.button('导出本次调试记录', on_click=export_diagnostics)
                ui.button('加载历史日志', on_click=load_saved_events).props('outline')
                alarm_filter = ui.select(['全部', '正在重试', '任务超时', '驱动器故障',
                                          '软件锁定', '任务失败', '反馈过期'], value='全部', label='报警筛选')
            ui.label('故障发生时的各关节快照；电流为协议值，请结合电流更新时间判断。')
            alarm_table = ui.table(columns=[{'name': k, 'field': k, 'label': label} for k, label in
                [('time', '发生时间'), ('category', '类别'), ('joint', '关节'), ('stage', '阶段'),
                 ('target', '目标 (°)'), ('position', '实际 (°)'), ('current', '电流'),
                 ('fault', '驱动故障'), ('updated', '状态更新时间'), ('position_updated', '位置更新时间'),
                 ('current_updated', '电流更新时间'), ('message', '详情')]],
                rows=[], pagination=10).classes('w-full')
            ui.label('最近 200 条事件；完整保留范围内的本次日志可导出。')
            event_log = ui.table(columns=[{'name': k, 'label': label, 'field': k}
                                          for k, label in [('time', '时间'), ('category', '类别'), ('stage', '阶段'), ('message', '事件')]],
                                 rows=[], pagination=15).classes('w-full')
for widget in motion_widgets + group_motion_widgets + commissioning_widgets + [enable_button, homing_button]:
    widget.disable()
ui.timer(0.5, refresh)
app.on_startup(startup)
app.on_shutdown(shutdown)
app.on_disconnect(disconnected)
ui.run(host=options.host, port=options.port, reload=False, show=False,
       title='DummyX 机械臂控制台')
