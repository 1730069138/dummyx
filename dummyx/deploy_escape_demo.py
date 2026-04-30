import mujoco
import mujoco.viewer
import numpy as np
import time
import argparse
import cv2
import threading
import queue
from collections import deque
from openpi_client import websocket_client_policy

# ==========================================
# 🧮 核心数学与运动学工具库
# ==========================================
def get_body_jacobian(model, data, body_id):
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, None, body_id)
    return jacp

def damped_pinv(J, rho=0.01):
    return J.T @ np.linalg.inv(J @ J.T + rho**2 * np.eye(J.shape[0]))

# ==========================================
# 🎲 场景初始化：极坐标域随机化与陷阱自适应旋转
# ==========================================
def reset_scene(model, data):
    base_x, base_y = 0.2, 0.2
    r_target = np.random.uniform(0.25, 0.35) 
    base_angle = -np.pi / 2 
    theta_target = base_angle + np.random.uniform(-np.pi/5, np.pi/5) 
    target_x = base_x + r_target * np.cos(theta_target)
    target_y = base_y + r_target * np.sin(theta_target)
    target_z = 0.22 

    target_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_joint")
    if target_jnt_id != -1:
        target_adr = model.jnt_qposadr[target_jnt_id]
        data.qpos[target_adr:target_adr+3] = [target_x, target_y, target_z]

    r_obs = r_target - 0.12 
    theta_obs = theta_target
    u_trap_x = base_x + r_obs * np.cos(theta_obs)
    u_trap_y = base_y + r_obs * np.sin(theta_obs)
    u_trap_z = 0.3 

    u_trap_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "u_trap")
    model.body_pos[u_trap_id] = [u_trap_x, u_trap_y, u_trap_z]
    
    half_theta = theta_obs / 2.0
    model.body_quat[u_trap_id] = [np.cos(half_theta), 0, 0, np.sin(half_theta)]
    mujoco.mj_forward(model, data)

# ==========================================
# 🌐 创新点 3：高层 10Hz VLA 推理后台线程
# ==========================================
def vla_inference_worker(policy, obs_queue, action_queue, trigger_event):
    action_chunk_buffer = deque()
    
    while True:
        obs = None
        while not obs_queue.empty():
            obs = obs_queue.get()
            
        if obs is None:
            time.sleep(0.01)
            continue

        if trigger_event.is_set():
            print("\n[High-Level 10Hz] 🛑 接收到底层卡死 Trigger！执行逃逸重规划...")
            action_chunk_buffer.clear() 
            trigger_event.clear()
            force_replan = True 
        else:
            force_replan = False

        if len(action_chunk_buffer) == 0:
            try:
                result = policy.infer(obs)
                actions = np.array(result["actions"]).copy() 
                
                if force_replan:
                    # 💡 逃逸扰动：只对前3维（位置）注入扰动，保留夹爪意图
                    actions[:, 0] += 0.04 
                    actions[:, 1] += np.random.choice([-0.12, 0.12]) 
                    actions[:, 2] += 0.18 # 稍微减小一点抬升高度，方便逃逸后快速回正

                for act in actions:
                    action_chunk_buffer.append(act)
            except Exception as e:
                print(f"[High-Level Error] 推理失败: {e}")
                time.sleep(0.1)

        if len(action_chunk_buffer) > 0:
            current_action = action_chunk_buffer.popleft()
            while not action_queue.empty():
                action_queue.get()
            action_queue.put(current_action)

        time.sleep(0.1)

# ==========================================
# 🛡️ 创新点 1 & 2：底层 100Hz 零空间护盾与极小值监控
# ==========================================
def apply_null_space_shield_and_monitor(model, data, current_qpos, target_action, obstacle_id, tcp_id, target_id, vel_history, trigger_event):
    q_dot_vla = target_action[:6] - current_qpos[:6]
    protected_bodies = ["link3_1", "link5_1", "link8"]
    obstacle_pos = data.xpos[obstacle_id]
    
    active_jacobians = []
    q_dot_avoid = np.zeros(6)
    
    for b_name in protected_bodies:
        b_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b_name)
        curr_pos = data.xpos[b_id]
        dist = np.linalg.norm(curr_pos - obstacle_pos)
        
        SAFE_DIST = 0.15 
        if dist < SAFE_DIST:
            n = (curr_pos - obstacle_pos) / dist
            rep_mag = 0.5 * (1.0/dist - 1.0/SAFE_DIST)**2
            
            tangent = np.cross(n, np.array([0, 0, 1]))
            if np.linalg.norm(tangent) > 1e-3:
                tangent = tangent / np.linalg.norm(tangent)
            v_avoid_task = (n * 0.2 + tangent * 0.1) * rep_mag
            
            J_b = get_body_jacobian(model, data, b_id)[:, :6]
            active_jacobians.append(J_b)
            q_dot_avoid += damped_pinv(J_b, 0.02) @ v_avoid_task

    if len(active_jacobians) > 0:
        J_safe = np.vstack(active_jacobians)
        J_safe_inv = J_safe.T @ np.linalg.inv(J_safe @ J_safe.T + 0.04**2 * np.eye(J_safe.shape[0]))
        P_safe = np.eye(6) - J_safe_inv @ J_safe
        q_dot_final = q_dot_avoid + P_safe @ q_dot_vla
    else:
        q_dot_final = q_dot_vla

    q_dot_final = np.clip(q_dot_final, -2.5, 2.5)
    
    v_norm = np.linalg.norm(q_dot_final)
    vel_history.append(v_norm)
    p_tcp = data.xpos[tcp_id]
    p_target = data.xpos[target_id]
    dist_to_target = np.linalg.norm(p_target - p_tcp)
    
    if len(vel_history) == vel_history.maxlen and dist_to_target > 0.1:
        avg_vel = sum(vel_history) / len(vel_history)
        if avg_vel < 0.05 and not trigger_event.is_set():
            print(f"[Low-Level 100Hz] ⚠️ 警告: 检测到势能极小值陷阱! 平均速度={avg_vel:.3f}")
            trigger_event.set() 
            vel_history.clear()

    return q_dot_final

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0", type=str)
    parser.add_argument("--port", default=8000, type=int)
    args = parser.parse_args()

    policy = websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
    print("✅ 服务端连接成功！")

    xml_path = "dummyx_apf_scene.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    data.ctrl[:] = data.qpos[:model.nu]

    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link8")
    obstacle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "u_trap")
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    cam_f_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed_cam")
    cam_w_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")
    renderer = mujoco.Renderer(model, height=256, width=256)

    reset_scene(model, data)

    obs_queue = queue.Queue(maxsize=2)
    action_queue = queue.Queue(maxsize=5)
    trigger_event = threading.Event()
    
    vla_thread = threading.Thread(
        target=vla_inference_worker, 
        args=(policy, obs_queue, action_queue, trigger_event),
        daemon=True
    )
    vla_thread.start()

    vel_history = deque(maxlen=50) 
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_action_full = np.zeros(8) # 💡 修改：初始化为 8 维
        
        while viewer.is_running():
            step_start = time.time()
            
            if np.random.rand() < 0.1:
                renderer.update_scene(data, camera=cam_f_id)
                img_f = renderer.render()
                renderer.update_scene(data, camera=cam_w_id)
                img_w = renderer.render()
                
                obs = {
                    "observation/image": img_f,
                    "observation/wrist_image": img_w,
                    "observation/state": data.qpos[:6].copy(),
                    "prompt": "Pick up the green box."
                }
                if not obs_queue.full():
                    obs_queue.put(obs)

            # 💡 核心修改：接收完整 8 维指令 (6臂 + 2爪)
            if not action_queue.empty():
                last_action_full = action_queue.get()

            # 3. 运行底层护盾 (仅针对前 6 维臂关节)
            q_dot_arm = apply_null_space_shield_and_monitor(
                model, data, data.qpos[:6], last_action_full[:6], 
                obstacle_id, tcp_id, target_id, vel_history, trigger_event
            )
            
            # 💡 核心下发逻辑：
            # 臂关节：当前角度 + 护盾处理后的增量
            data.ctrl[:6] = data.qpos[:6] + q_dot_arm * model.opt.timestep
            # 夹爪：直接透传 VLA 输出的第 7, 8 位指令
            data.ctrl[6:8] = last_action_full[6:8]
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))

if __name__ == "__main__":
    main()