import mujoco
import mujoco.viewer
import numpy as np
import time

def get_body_jacobian(model, data, body_id):
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, None, body_id)
    return jacp

def damped_pinv(J, rho=0.01):
    return J.T @ np.linalg.inv(J @ J.T + rho**2 * np.eye(J.shape[0]))

def main():
    xml_path = "dummyx_apf_scene.xml" 
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 重点监控连杆
    protected_bodies = ["link3_1", "link5_1", "link8"]
    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in protected_bodies]
    
    tcp_id = body_ids[-1] 
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    obstacle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "obstacle")

    mujoco.mj_forward(model, data)
    data.ctrl[:] = data.qpos[:model.nu] 
    dt = model.opt.timestep
    
    # --- 调优参数：打破死锁的关键 ---
    D_SAFE_BASE = 0.1   # 基础安全半径
    BETA_SAFE = 0.3      # 提高避障推力响应
    K_VLA = 0.12         # 提高目标吸引速度（从0.08增加到0.12）
    GAMMA_FORCE = 0.4    # ⚠️ 强化切向力，强制破除顶牛状态

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            target_pos = data.xpos[target_id]
            obs_pos = data.xpos[obstacle_id]
            tcp_pos = data.xpos[tcp_id]

            # 1. 目标吸引任务
            dist_to_target = np.linalg.norm(target_pos - tcp_pos)
            v_vla = (target_pos - tcp_pos)
            v_vla = (v_vla / dist_to_target) * K_VLA if dist_to_target > 0.01 else np.zeros(3)

            # 2. 动态调整安全半径 (创新点：当靠近目标时，适当缩小安全区以允许精准抓取)
            # 这能解决“因为目标在障碍物附近而无法靠近”的问题
            d_safe_eff = min(D_SAFE_BASE, dist_to_target * 0.8 + 0.05)

            active_jacobians = []
            q_dot_avoid = np.zeros(6)

            for b_id in body_ids:
                curr_pos = data.xpos[b_id]
                vec_obs_to_body = curr_pos - obs_pos
                vec_obs_to_body[2] = 0 # 约束在 XY 平面避障
                dist = np.linalg.norm(vec_obs_to_body)

                if dist < d_safe_eff:
                    n = vec_obs_to_body / dist
                    rep_mag = (d_safe_eff - dist) / d_safe_eff
                    
                    # ⚠️ 强化涡旋分量：强制产生侧向滑行
                    tangent = np.array([-n[1], n[0], 0])
                    # 引导方向始终朝向目标一侧
                    if np.dot(tangent, target_pos - curr_pos) < 0:
                        tangent = -tangent
                    
                    v_avoid_task = (n * 0.2 + tangent * GAMMA_FORCE) * rep_mag
                    
                    J_b = get_body_jacobian(model, data, b_id)[:, :6]
                    active_jacobians.append(J_b)
                    q_dot_avoid += damped_pinv(J_b, 0.02) @ v_avoid_task

            # 3. 零空间投影融合
            if len(active_jacobians) > 0:
                J_safe = np.vstack(active_jacobians)
                # 投影矩阵：保护安全动作不被主任务干扰
                J_safe_inv = J_safe.T @ np.linalg.inv(J_safe @ J_safe.T + 0.04**2 * np.eye(J_safe.shape[0]))
                P_safe = np.eye(6) - J_safe_inv @ J_safe
                
                J_tcp = get_body_jacobian(model, data, tcp_id)[:, :6]
                q_dot_vla = damped_pinv(J_tcp, 0.05) @ v_vla
                
                # 最终动作 = 避障(优先级高) + 投影后的目标任务
                q_dot_final = q_dot_avoid + P_safe @ q_dot_vla
            else:
                J_tcp = get_body_jacobian(model, data, tcp_id)[:, :6]
                q_dot_final = damped_pinv(J_tcp, 0.05) @ v_vla

            # 4. 执行控制与限幅
            q_dot_final = np.clip(q_dot_final, -2.0, 2.0)
            data.ctrl[:6] = data.ctrl[:6] + q_dot_final * dt
            
            mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.time() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

if __name__ == "__main__":
    main()
