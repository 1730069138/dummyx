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

def reset_scene(model, data):
    """
    混合传送魔法：目标点用 body_pos (修改幻影位置)，障碍物用 qpos (传送物理刚体)
    """
    base_x, base_y = 0.2, 0.2
    
    # 1. 目标点生成坐标
    r_target = np.random.uniform(0.25, 0.35) 
    base_angle = -np.pi / 2 
    theta_target = base_angle + np.random.uniform(-np.pi/5, np.pi/5) 
    target_x = base_x + r_target * np.cos(theta_target)
    target_y = base_y + r_target * np.sin(theta_target)
    target_z = 0.22 

    # 💡 目标点现在是幻影，直接修改 XML 的基础坐标
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    model.body_pos[target_id] = [target_x, target_y, target_z]

    # 2. 障碍物生成坐标
    ratio = np.random.uniform(0.45, 0.65)
    obs_x = base_x + ratio * (target_x - base_x) + np.random.uniform(-0.06, 0.06)
    obs_y = base_y + ratio * (target_y - base_y) + np.random.uniform(-0.06, 0.06)
    
    dist_to_base = np.hypot(obs_x - base_x, obs_y - base_y)
    if dist_to_base < 0.15:
        obs_x = base_x + ((obs_x - base_x) / dist_to_base) * 0.15
        obs_y = base_y + ((obs_y - base_y) / dist_to_base) * 0.15
    obs_z = 0.30 

    # 3. 清空所有物理状态 (速度和关节角归零)
    data.qpos[:] = 0
    data.qvel[:] = 0 
    data.ctrl[:] = 0
    
    # 4. 💡 仅对障碍圆柱使用 qpos 瞬间传送
    obs_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "obstacle_joint")
    obs_adr = model.jnt_qposadr[obs_jnt_id]
    data.qpos[obs_adr : obs_adr+3] = [obs_x, obs_y, obs_z]
    data.qpos[obs_adr+3 : obs_adr+7] = [1, 0, 0, 0]

    mujoco.mj_forward(model, data)

def main():
    xml_path = "dummyx_apf_scene.xml" 
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    protected_bodies = ["link3_1", "link5_1", "link7", "link8"]
    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in protected_bodies]
    tcp_id = body_ids[-1] 
    
    # 依然获取 Body ID 用于测距
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    obstacle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "obstacle")

    D_SAFE_BASE = 0.08   
    K_VLA = 0.15         
    GAMMA_FORCE = 0.6    
    TIME_LIMIT = 15.0    
    
    total_episodes = 20
    success_count = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        for episode in range(total_episodes):
            print(f"\n--- 正在开始第 {episode + 1} 轮实验 ---")
            
            # 注意：这里的传参精简了，因为重置逻辑在内部通过 joint name 查找了
            reset_scene(model, data)
            
            episode_start = time.time()
            success = False
            
            while viewer.is_running() and (time.time() - episode_start) < TIME_LIMIT:
                step_start = time.time()
                
                # 实时读取动态物理位置
                target_pos = data.xpos[target_id]
                obs_pos = data.xpos[obstacle_id]
                tcp_pos = data.xpos[tcp_id]

                # 💡 新增：物体掉下桌子 (Z 轴高度极速下降) 视为失败
                if target_pos[2] < 0.15:
                    print("❌ 目标掉下桌子，任务失败！")
                    break

                dist_to_target = np.linalg.norm(target_pos - tcp_pos)
                if dist_to_target < 0.03:
                    print("✅ 抓取成功！")
                    success = True
                    success_count += 1
                    break

                v_vla = (target_pos - tcp_pos)
                v_vla = (v_vla / dist_to_target) * K_VLA if dist_to_target > 0.01 else np.zeros(3)

                d_safe_eff = min(D_SAFE_BASE, dist_to_target * 0.8 + 0.03) 

                active_jacobians = []
                q_dot_avoid = np.zeros(6)

                for b_id in body_ids:
                    curr_pos = data.xpos[b_id]
                    vec_obs_to_body = curr_pos - obs_pos
                    vec_obs_to_body[2] = 0 
                    dist = np.linalg.norm(vec_obs_to_body)

                    if dist < d_safe_eff:
                        n = vec_obs_to_body / dist
                        rep_mag = (d_safe_eff - dist) / d_safe_eff
                        
                        tangent = np.array([-n[1], n[0], 0])
                        if np.dot(tangent, target_pos - curr_pos) < 0:
                            tangent = -tangent
                        
                        v_avoid_task = (n * 0.15 + tangent * GAMMA_FORCE) * rep_mag
                        
                        J_b = get_body_jacobian(model, data, b_id)[:, :6]
                        active_jacobians.append(J_b)
                        q_dot_avoid += damped_pinv(J_b, 0.03) @ v_avoid_task 

                if len(active_jacobians) > 0:
                    J_safe = np.vstack(active_jacobians)
                    J_safe_inv = J_safe.T @ np.linalg.inv(J_safe @ J_safe.T + 0.05**2 * np.eye(J_safe.shape[0]))
                    P_safe = np.eye(6) - J_safe_inv @ J_safe
                    
                    J_tcp = get_body_jacobian(model, data, tcp_id)[:, :6]
                    q_dot_vla = damped_pinv(J_tcp, 0.05) @ v_vla
                    
                    q_dot_final = q_dot_avoid + P_safe @ q_dot_vla
                else:
                    J_tcp = get_body_jacobian(model, data, tcp_id)[:, :6]
                    q_dot_final = damped_pinv(J_tcp, 0.05) @ v_vla

                q_dot_final = np.clip(q_dot_final, -2.5, 2.5)
                data.ctrl[:6] = data.ctrl[:6] + q_dot_final * model.opt.timestep
                
                mujoco.mj_step(model, data)
                viewer.sync()

            if not success:
                print("❌ 任务终止")
                
            time.sleep(0.5)

    print(f"\n测试结束！总次数: {total_episodes}, 成功次数: {success_count}, 成功率: {success_count/total_episodes*100:.1f}%")

if __name__ == "__main__":
    main()