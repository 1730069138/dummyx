import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import cv2

def get_site_jacobian(model, data, site_id):
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, None, site_id)
    return jacp

def damped_pinv(J, rho=0.01):
    return J.T @ np.linalg.inv(J @ J.T + rho**2 * np.eye(J.shape[0]))

def reset_scene(model, data):
    # 在绿毯可视化区域(spawn_area)内随机生成螺丝刀的位置
    target_x = np.random.uniform(0.20, 0.35)    
    target_y = np.random.uniform(-0.15, -0.05)  
    target_z = 0.22

    # 💡 核心修改：同步 XML 中全新的螺丝刀关节名称
    target_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "fj_screwdriver")
    if target_jnt_id != -1:
        target_adr = model.jnt_qposadr[target_jnt_id]
        data.qpos[target_adr : target_adr+3] = [target_x, target_y, target_z]
        # 保持四元数为 [1,0,0,0]，确保螺丝刀姿态始终与夹爪方向对齐
        data.qpos[target_adr+3 : target_adr+7] = [1, 0, 0, 0]
    else:
        target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "real_screwdriver")
        model.body_pos[target_id] = [target_x, target_y, target_z]

    # 隐藏障碍物红柱子，保持环境纯净
    obs_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "obstacle_joint")
    if obs_jnt_id != -1:
        obs_adr = model.jnt_qposadr[obs_jnt_id]
        data.qpos[obs_adr : obs_adr+3] = [10.0, 10.0, -10.0]
        data.qpos[obs_adr+3 : obs_adr+7] = [1, 0, 0, 0]
    else:
        obs_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "obstacle")
        model.body_pos[obs_id] = [10.0, 10.0, -10.0]

    data.qpos[:8] = 0
    data.qvel[:] = 0
    data.ctrl[:] = 0
    mujoco.mj_forward(model, data)

def main():
    xml_path = "dummyx_apf_scene.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    renderer = mujoco.Renderer(model, height=256, width=256)
    save_dir = "dataset_screwdriver_insertion" 
    os.makedirs(save_dir, exist_ok=True)

    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp_site")
    
    # 💡 核心修改：同步 XML 中全新的螺丝刀 Body 名称
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "real_screwdriver")
    
    # 获取绿毯区域 ID
    spawn_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "spawn_area")

    GRIPPER_OPEN = 0.04   
    GRIPPER_CLOSE = 0.0    
    
    TARGET_SUCCESS_EPISODES = 50 
    fps = 10
    sim_steps_per_record = int((1.0 / fps) / model.opt.timestep)
    
    # 💡 核心修改：全面替换语义 Prompt，与螺丝刀对齐
    language_instructions = [
        "Pick up the screwdriver and drop it into the box.",
        "Insert the screwdriver into the receptacle.",
        "Grasp the flat screwdriver and place it securely inside the container.",
        "Put the screwdriver in the target box."
    ]
    
    print(f"🚀 启动 [抓取螺丝刀入盒] 专家采集模式！目标: {TARGET_SUCCESS_EPISODES} 条...")

    success_count = 0
    total_attempts = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and success_count < TARGET_SUCCESS_EPISODES:
            total_attempts += 1
            reset_scene(model, data)
            
            phase = "HOVER_A"
            grasp_record_steps = 0
            release_record_steps = 0
            success = False
            
            ep_images_fixed, ep_images_wrist = [], []
            ep_qpos, ep_actions = [], []
            step_counter = 0
            
            # 保持目标盒子位置为最佳坐标
            drop_point_b = np.array([0.0, -0.1, 0.22])
            
            print(f"\n--- 尝试第 {total_attempts} 轮 (已收集: {success_count}/{TARGET_SUCCESS_EPISODES}) ---")

            while viewer.is_running():
                target_pos = data.xpos[target_id]
                tcp_pos = data.site_xpos[tcp_id]
                
                # A 点的参考坐标 (螺丝刀重心处)
                hover_point_a = target_pos + np.array([0.0, 0.0, 0.12])
                # 💡 核心修改：将 Z 轴偏移量从 0.001 改为 -0.015，使夹爪下潜更深，避免只捏到边缘
                grasp_point_a = target_pos + np.array([0.0, 0.0, -0.015])
                
                # B 点的参考坐标 (盒子处)
                hover_point_b = drop_point_b + np.array([0.0, 0.0, 0.18])
                release_point_b = drop_point_b + np.array([0.0, 0.0, 0.03])

                q_dot_final = np.zeros(6)
                gripper_target = GRIPPER_OPEN

                # ==========================================
                # 🧠 8 阶段专家状态机 (Box Insertion)
                # ==========================================
                
                if phase == "HOVER_A":
                    dist = np.linalg.norm(hover_point_a - tcp_pos)
                    if dist < 0.02:
                        phase = "DESCEND_A"
                        print("👉 1. 到达螺丝刀上方，开始下降...")
                    else:
                        v_vla = (hover_point_a - tcp_pos) / dist * 0.15
                        J_tcp = get_site_jacobian(model, data, tcp_id)[:, :6]
                        q_dot_final = damped_pinv(J_tcp, 0.05) @ v_vla

                elif phase == "DESCEND_A":
                    dist = np.linalg.norm(grasp_point_a - tcp_pos)
                    if dist < 0.01:
                        phase = "GRASP"
                        print("👉 2. 贴近螺丝刀中心，闭合夹爪！")
                    else:
                        v_vla = (grasp_point_a - tcp_pos) / dist * 0.05 
                        J_tcp = get_site_jacobian(model, data, tcp_id)[:, :6]
                        q_dot_final = damped_pinv(J_tcp, 0.05) @ v_vla

                elif phase == "GRASP":
                    q_dot_final = np.zeros(6) 
                    gripper_target = GRIPPER_CLOSE 
                    if step_counter % sim_steps_per_record == 0:
                        grasp_record_steps += 1
                    if grasp_record_steps >= int(1.5 * fps):
                        phase = "LIFT_A"
                        fixed_lift_point_a = tcp_pos.copy() + np.array([0.0, 0.0, 0.18])
                        print("👉 3. 成功夹住螺丝刀，高空平滑提起...")

                elif phase == "LIFT_A":
                    gripper_target = GRIPPER_CLOSE 
                    dist = np.linalg.norm(fixed_lift_point_a - tcp_pos)
                    if dist < 0.03:
                        phase = "MOVE_TO_B"
                        print("👉 4. 提起完成，向目标盒子平移...")
                    else:
                        v_vla = (fixed_lift_point_a - tcp_pos) / dist * 0.08
                        J_tcp = get_site_jacobian(model, data, tcp_id)[:, :6]
                        q_dot_final = damped_pinv(J_tcp, 0.05) @ v_vla

                elif phase == "MOVE_TO_B":
                    gripper_target = GRIPPER_CLOSE
                    current_hover_b = np.array([hover_point_b[0], hover_point_b[1], tcp_pos[2]])
                    dist = np.linalg.norm(current_hover_b - tcp_pos)
                    if dist < 0.03:
                        phase = "DESCEND_B"
                        print("👉 5. 到达盒子正上方，垂直深潜入盒...")
                    else:
                        v_vla = (current_hover_b - tcp_pos) / dist * 0.10
                        J_tcp = get_site_jacobian(model, data, tcp_id)[:, :6]
                        q_dot_final = damped_pinv(J_tcp, 0.05) @ v_vla

                elif phase == "DESCEND_B":
                    gripper_target = GRIPPER_CLOSE
                    dist = np.linalg.norm(release_point_b - tcp_pos)
                    if dist < 0.02:
                        phase = "RELEASE"
                        print("👉 6. 深入盒子底部，松开夹爪！")
                    else:
                        v_vla = (release_point_b - tcp_pos) / dist * 0.05
                        J_tcp = get_site_jacobian(model, data, tcp_id)[:, :6]
                        q_dot_final = damped_pinv(J_tcp, 0.05) @ v_vla

                elif phase == "RELEASE":
                    q_dot_final = np.zeros(6) 
                    gripper_target = GRIPPER_OPEN 
                    if step_counter % sim_steps_per_record == 0:
                        release_record_steps += 1
                    if release_record_steps >= int(1.0 * fps):
                        phase = "LIFT_B"
                        fixed_lift_point_b = tcp_pos.copy() + np.array([0.0, 0.0, 0.15])
                        print("👉 7. 螺丝刀落袋，垂直原路向上抽离...")

                elif phase == "LIFT_B":
                    gripper_target = GRIPPER_OPEN
                    dist = np.linalg.norm(fixed_lift_point_b - tcp_pos)
                    if dist < 0.03:
                        print("✅ 8. 抓取螺丝刀入盒任务圆满完成！")
                        success = True
                        break
                    else:
                        v_vla = (fixed_lift_point_b - tcp_pos) / dist * 0.10
                        J_tcp = get_site_jacobian(model, data, tcp_id)[:, :6]
                        q_dot_final = damped_pinv(J_tcp, 0.05) @ v_vla

                q_dot_final = np.clip(q_dot_final, -2.5, 2.5)

                # ==========================================
                # 💾 录制动作帧与图像
                # ==========================================
                if step_counter % sim_steps_per_record == 0:
                    
                    # 光学隐身术
                    if spawn_geom_id != -1:
                        model.geom_rgba[spawn_geom_id] = [0, 0, 0, 0]
                        
                    # 渲染机位使用 rear_cam (正后方越肩俯视)
                    renderer.update_scene(data, camera="rear_cam")
                    img_f = renderer.render()
                    renderer.update_scene(data, camera="wrist_cam")
                    img_w = renderer.render()
                    
                    # 光学恢复术
                    if spawn_geom_id != -1:
                        model.geom_rgba[spawn_geom_id] = [0, 1, 0, 0.3]
                    
                    current_qpos = data.qpos[:8].copy()
                    expert_action = np.zeros(8)
                    expert_action[:6] = current_qpos[:6] + q_dot_final * (1.0 / fps)
                    expert_action[6] = gripper_target
                    expert_action[7] = gripper_target
                    
                    ep_images_fixed.append(img_f)
                    ep_images_wrist.append(img_w)
                    ep_qpos.append(current_qpos)
                    ep_actions.append(expert_action)

                # ==========================================
                # ⚙️ 物理引擎步进执行
                # ==========================================
                data.ctrl[:6] = data.ctrl[:6] + q_dot_final * model.opt.timestep
                data.ctrl[6] = gripper_target
                data.ctrl[7] = gripper_target
                
                mujoco.mj_step(model, data)
                viewer.sync()
                step_counter += 1

                if step_counter > 60000:
                    print("❌ 动作超时，跳过本轮。")
                    break

            if success:
                ep_name = f"ep_{success_count:04d}_{time.strftime('%H%M%S')}"
                ep_dir = os.path.join(save_dir, ep_name)
                cam_f_dir = os.path.join(ep_dir, "cam_fixed")
                cam_w_dir = os.path.join(ep_dir, "cam_wrist")
                
                os.makedirs(cam_f_dir, exist_ok=True)
                os.makedirs(cam_w_dir, exist_ok=True)

                for i, (img_f, img_w) in enumerate(zip(ep_images_fixed, ep_images_wrist)):
                    cv2.imwrite(os.path.join(cam_f_dir, f"{i:03d}.jpg"), cv2.cvtColor(img_f, cv2.COLOR_RGB2BGR))
                    cv2.imwrite(os.path.join(cam_w_dir, f"{i:03d}.jpg"), cv2.cvtColor(img_w, cv2.COLOR_RGB2BGR))

                np.savez_compressed(
                    os.path.join(ep_dir, "joint_data.npz"),
                    qpos=np.array(ep_qpos, dtype=np.float32),
                    actions=np.array(ep_actions, dtype=np.float32)
                )
                
                chosen_instruction = np.random.choice(language_instructions)
                with open(os.path.join(ep_dir, "instruction.txt"), "w", encoding="utf-8") as f:
                    f.write(chosen_instruction)
                
                success_count += 1
                print(f"📁 已保存至 {ep_name}，当前总计: {success_count}/{TARGET_SUCCESS_EPISODES}")

    print(f"\n🎉 大功告成！完美收集 {TARGET_SUCCESS_EPISODES} 条抓取螺丝刀入盒数据！")

if __name__ == "__main__":
    main()