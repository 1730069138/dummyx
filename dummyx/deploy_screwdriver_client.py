import mujoco
import mujoco.viewer
import numpy as np
import time
import argparse
import cv2
import os
import open3d as o3d
import glfw

# ==========================================
# 👁️‍🗨️ 3D 视觉感知管线
# ==========================================
def get_camera_intrinsics(model, cam_name, height, width):
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
    fovy = model.cam_fovy[cam_id]
    f = 0.5 * height / np.tan(fovy * np.pi / 360)
    return o3d.camera.PinholeCameraIntrinsic(width, height, f, f, width/2, height/2)

def depth_to_point_cloud(model, data, depth_img, cam_name, intrinsics):
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
    cam_pos = data.cam_xpos[cam_id]
    cam_mat = data.cam_xmat[cam_id].reshape(3, 3)
    R_cv2mujoco = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    cam_to_world = np.eye(4)
    cam_to_world[:3, :3] = cam_mat @ R_cv2mujoco
    cam_to_world[:3, 3] = cam_pos
    depth_o3d = o3d.geometry.Image(depth_img.astype(np.float32))
    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_o3d, intrinsics)
    pcd.transform(cam_to_world)
    return pcd

def extract_obstacle_capsule(pcd, table_z=0.2):
    if pcd is None or len(pcd.points) == 0:
        return None
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.array([0.085, -0.20, table_z + 0.015]),
        max_bound=np.array([0.180,  0.05, table_z + 0.300])
    )
    pcd = pcd.crop(bbox)
    pcd = pcd.voxel_down_sample(voxel_size=0.005)
    if len(pcd.points) < 20:
        return None
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    labels = np.array(pcd.cluster_dbscan(eps=0.03, min_points=10))
    if len(labels) == 0 or labels.max() < 0:
        return None
    valid_clusters = []
    for i in range(labels.max() + 1):
        cluster_points = pcd.select_by_index(np.where(labels == i)[0])
        if len(cluster_points.points) < 15:
            continue
        obs_bbox = cluster_points.get_axis_aligned_bounding_box()
        extent = obs_bbox.get_extent()
        if extent[0] > 0.12 or extent[1] > 0.12:
            continue
        valid_clusters.append((len(cluster_points.points), cluster_points, obs_bbox))
    if not valid_clusters:
        return None
    valid_clusters.sort(key=lambda x: x[0], reverse=True)
    best_cluster = valid_clusters[0]
    obs_bbox = best_cluster[2]
    min_pt = obs_bbox.get_min_bound()
    max_pt = obs_bbox.get_max_bound()
    center = (min_pt + max_pt) / 2.0
    raw_radius = max((max_pt[0] - min_pt[0]), (max_pt[1] - min_pt[1])) / 2.0
    final_radius = np.clip(raw_radius + 0.02, 0.035, 0.05)
    p1 = np.array([center[0], center[1], min_pt[2]])
    p2 = np.array([center[0], center[1], max_pt[2]])
    return {"p1": p1, "p2": p2, "r": final_radius}

# ==========================================
# 🧮 核心数学
# ==========================================
def get_body_jacobian(model, data, body_id):
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, None, body_id)
    return jacp

def damped_pinv(J, rho=0.10):
    return J.T @ np.linalg.inv(J @ J.T + rho**2 * np.eye(J.shape[0]))

def segment_to_segment_distance(p1, p2, p3, p4):
    p1, p2, p3, p4 = map(np.array, (p1, p2, p3, p4))
    d1, d2, r = p2-p1, p4-p3, p1-p3
    a, e, f = np.dot(d1, d1), np.dot(d2, d2), np.dot(d2, r)
    if a <= 1e-6 and e <= 1e-6:
        s, t = 0.0, 0.0
    elif a <= 1e-6:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = np.dot(d1, r)
        if e <= 1e-6:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        else:
            b = np.dot(d1, d2)
            denom = a * e - b * b
            if denom != 0:
                s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    c1, c2 = p1 + d1 * s, p3 + d2 * t
    vec = c1 - c2
    dist = np.linalg.norm(vec)
    normal = vec / dist if dist > 1e-6 else np.array([1.0, 0.0, 0.0])
    return dist, normal, c1, c2

# ==========================================
# 🚨 新增模块1：轨迹安全检查
# ==========================================
# 【优化】安全裕度微调，减少不必要的轨迹替换拖慢动作
def check_trajectory_safety(model, data, action_chunk, obstacle_capsule, link6_id, tcp_id, safe_margin=0.015):
    if obstacle_capsule is None:
        return False, -1
    temp_data = mujoco.MjData(model)
    temp_data.qpos[:] = data.qpos[:].copy()
    temp_data.qvel[:] = data.qvel[:].copy()
    mujoco.mj_forward(model, temp_data)
    for frame_idx, action in enumerate(action_chunk):
        temp_data.qpos[:6] = action[:6]
        mujoco.mj_forward(model, temp_data)
        wrist_pos = temp_data.xpos[link6_id].copy()
        tcp_pos = temp_data.site_xpos[tcp_id].copy()
        ee_capsule = {"p1": wrist_pos, "p2": tcp_pos, "r": 0.04}
        dist, _, _, _ = segment_to_segment_distance(
            ee_capsule["p1"], ee_capsule["p2"],
            obstacle_capsule["p1"], obstacle_capsule["p2"]
        )
        actual_dist = dist - ee_capsule["r"] - obstacle_capsule["r"]
        if actual_dist < safe_margin:
            return True, frame_idx
    return False, -1

# ==========================================
# 🛡️ 新增模块2：APF全局轨迹生成
# ==========================================
# 【优化】step_size从0.02→0.05，TCP移动步长翻倍，大幅加快动作速度
def generate_apf_global_trajectory(model, data, target_pos, obstacle_capsule, link6_id, tcp_id, num_frames=8, step_size=0.05):
    safe_tcp_traj = []
    current_tcp = data.site_xpos[tcp_id].copy()
    for _ in range(num_frames):
        goal_dir = target_pos - current_tcp
        goal_dist = np.linalg.norm(goal_dir)
        if goal_dist > 1e-6:
            goal_dir /= goal_dist
        force_attr = goal_dir * min(goal_dist, step_size)
        force_rep = np.zeros(3)
        if obstacle_capsule is not None:
            wrist_pos = data.xpos[link6_id].copy()
            ee_cap = {"p1": wrist_pos, "p2": current_tcp, "r": 0.04}
            dist, normal, _, _ = segment_to_segment_distance(
                ee_cap["p1"], ee_cap["p2"],
                obstacle_capsule["p1"], obstacle_capsule["p2"]
            )
            actual_dist = dist - ee_cap["r"] - obstacle_capsule["r"]
            dynamic_safe_margin = np.clip(0.015 + (goal_dist * 0.2), 0.015, 0.05)
            if actual_dist < dynamic_safe_margin:
                influence = np.power((dynamic_safe_margin - max(actual_dist, 0.0)) / dynamic_safe_margin, 2)
                force_rep = normal * (influence * 0.05)
        total_force = force_attr + force_rep
        next_tcp = current_tcp + total_force
        safe_tcp_traj.append(next_tcp.copy())
        current_tcp = next_tcp
    return safe_tcp_traj

# ==========================================
# 🔄 新增模块3：逆运动学 IK
# ==========================================
# 【优化】迭代次数从20→40，精度从1e-4→1e-5，求解更精准，减少无效修正步
def solve_ik(model, data, target_tcp_pos, tcp_id, max_iter=40, tol=1e-5):
    target_qpos = data.qpos[:6].copy()
    temp_data = mujoco.MjData(model)
    temp_data.qpos[:] = data.qpos[:].copy()
    for _ in range(max_iter):
        temp_data.qpos[:6] = target_qpos
        mujoco.mj_forward(model, temp_data)
        current_tcp = temp_data.site_xpos[tcp_id].copy()
        error = target_tcp_pos - current_tcp
        if np.linalg.norm(error) < tol:
            break
        J = get_body_jacobian(model, temp_data, tcp_id)[:3, :6]
        J_pinv = damped_pinv(J, rho=0.05)
        delta_q = J_pinv @ error
        target_qpos += delta_q
        target_qpos = np.clip(target_qpos, -np.pi, np.pi)
    return target_qpos

# ==========================================
# 🎨 新增模块4：轨迹平滑
# ==========================================
# 【优化】lpf_beta从0.7→0.9，减少平滑延迟，动作更跟手、更快
def smooth_trajectory(current_qpos, target_q_traj, lpf_beta=0.9):
    smoothed_traj = []
    last_q = current_qpos.copy()
    for target_q in target_q_traj:
        smoothed_q = lpf_beta * target_q + (1 - lpf_beta) * last_q
        smoothed_traj.append(smoothed_q)
        last_q = smoothed_q
    return smoothed_traj

# ==========================================
# 🛡️ 原有 APF 后处理（保留）
# ==========================================
last_q_dot = np.zeros(6)
def process_apf_action(model, data, current_action, perceived_obstacle_capsule, step_counter):
    global last_q_dot
    current_qpos = data.qpos[:8].copy()
    q_dot_vla = current_action[:6] - current_qpos[:6]
    raw_gripper = current_action[6]
    gripper_val = 0.04 if raw_gripper > 0.02 else 0.0
    link6_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6_1")
    link7_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link7")
    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp_site")
    screw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "real_screwdriver")
    tcp_pos = data.site_xpos[tcp_id].copy()
    box_pos = np.array([0.0, -0.1, 0.24])
    screw_pos = data.xpos[screw_id].copy()
    is_grasped = (gripper_val < 0.02 and np.linalg.norm(tcp_pos - screw_pos) < 0.05)
    if not is_grasped:
        active_target_pos = screw_pos
    else:
        active_target_pos = box_pos
    dist_to_target = np.linalg.norm(tcp_pos - active_target_pos)
    dynamic_safe_margin = np.clip(0.015 + (dist_to_target * 0.2), 0.015, 0.05)
    wrist_pos = data.xpos[link6_id].copy()
    active_capsules = [{"p1": wrist_pos, "p2": tcp_pos, "r": 0.04, "body_id": link7_id}]
    delta_q_apf = np.zeros(6)
    is_apf_active = False
    if perceived_obstacle_capsule is not None:
        min_dist = float('inf')
        dom_normal, dom_body_id = None, None
        for cap in active_capsules:
            dist, normal, _, _ = segment_to_segment_distance(cap["p1"], cap["p2"], perceived_obstacle_capsule["p1"], perceived_obstacle_capsule["p2"])
            actual_dist = dist - cap["r"] - perceived_obstacle_capsule["r"]
            if actual_dist < min_dist:
                min_dist, dom_normal, dom_body_id = actual_dist, normal, cap["body_id"]
        if min_dist < dynamic_safe_margin:
            influence = np.power((dynamic_safe_margin - max(min_dist, 0.0)) / dynamic_safe_margin, 2)
            if influence > 0.005:
                is_apf_active = True
                J_dom = get_body_jacobian(model, data, dom_body_id)[:3, :6]
                v_vla_cartesian = J_dom @ q_dot_vla
                v_into_obs = np.dot(v_vla_cartesian, dom_normal)
                if v_into_obs < 0:
                    v_vla_filtered = v_vla_cartesian - v_into_obs * dom_normal
                else:
                    v_vla_filtered = v_vla_cartesian + dom_normal * (influence * 0.002)
                delta_v = v_vla_filtered - v_vla_cartesian
                delta_q_apf = damped_pinv(J_dom, 0.05) @ delta_v
    q_dot_combined = q_dot_vla + delta_q_apf
    q_dot_smooth = 0.6 * q_dot_combined + 0.4 * last_q_dot
    last_q_dot = q_dot_smooth.copy()
    final_ctrl = np.zeros(8)
    final_ctrl[:6] = current_qpos[:6] + np.clip(q_dot_smooth, -0.25, 0.25)
    final_ctrl[6:8] = gripper_val
    return final_ctrl, active_capsules, is_apf_active

# ==========================================
# 🌍 场景复位
# ==========================================
def reset_scene(model, data, use_obstacle=False):
    target_x = np.random.uniform(0.20, 0.35)
    target_y = np.random.uniform(-0.15, -0.05)
    target_z = 0.22
    target_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "fj_screwdriver")
    if target_jnt_id != -1:
        target_adr = model.jnt_qposadr[target_jnt_id]
        data.qpos[target_adr:target_adr+3] = [target_x, target_y, target_z]
        data.qpos[target_adr+3:target_adr+7] = [1,0,0,0]
    pillar_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pillar_joint")
    if pillar_jnt_id != -1:
        q_adr = model.jnt_qposadr[pillar_jnt_id]
        if use_obstacle:
            data.qpos[q_adr:q_adr+3] = [0.11, -0.1, 0.38]
            data.qpos[q_adr+3:q_adr+7] = [1,0,0,0]
        else:
            data.qpos[q_adr:q_adr+3] = [10,10,-10]
    data.qpos[:8] = 0
    data.qvel[:8] = 0
    data.ctrl[:] = 0
    mujoco.mj_forward(model, data)

def save_episode_video(frames, folder, episode_idx):
    if not frames:
        return
    os.makedirs(folder, exist_ok=True)
    filename = os.path.join(folder, f"ep_{episode_idx:03d}.mp4")
    h, w = frames[0].shape[:2]
    out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'mp4v'), 10, (w, h))
    for f in frames:
        out.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
    out.release()

# ==========================================
# 🚀 主程序（全优化版）
# ==========================================
def main(args):
    if not glfw.init():
        raise RuntimeError("GLFW 初始化失败")

    xml_path = "dummyx_apf_scene.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    renderer_rgb = mujoco.Renderer(model, 256, 256)
    depth_h, depth_w = 480, 640
    renderer_depth = mujoco.Renderer(model, depth_h, depth_w)
    renderer_depth.enable_depth_rendering()
    intrinsics = get_camera_intrinsics(model, "rear_cam", depth_h, depth_w)

    from openpi_client import websocket_client_policy
    policy = websocket_client_policy.WebsocketClientPolicy(args.host, args.port)

    link6_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6_1")
    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp_site")
    target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "real_screwdriver")
    pillar_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "dynamic_pillar")

    success = 0
    collision = 0
    replace_count = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.sync()
        for ep in range(100):
            if not viewer.is_running():
                break
            reset_scene(model, data, args.use_obstacle)
            viewer.sync()

            step = 0
            in_box = 0
            ok = False
            col = False
            cache = None
            safe_cache = None
            last_obs = None
            frames = []
            qhist = []

            # 【核心优化1】总步数从600→1500，彻底解决没动完就结束的问题
            while viewer.is_running() and step < 1500:
                qhist.append(data.qpos[:6].copy())

                # 【核心优化2】动作更新频率从8步→4步，刷新更快，动作不卡顿
                if step % 4 == 0 or cache is None:
                    renderer_rgb.update_scene(data, camera="rear_cam")
                    img_f = renderer_rgb.render()
                    renderer_rgb.update_scene(data, camera="wrist_cam")
                    img_w = renderer_rgb.render()

                    obs = {
                        "observation/image": img_f,
                        "observation/wrist_image": img_w,
                        "observation/state": data.qpos[:8].copy(),
                        "prompt": "Pick up the screwdriver and drop it into the box."
                    }
                    res = policy.infer(obs)
                    cache = res["actions"]
                    safe_cache = [a.copy() for a in cache]

                    # 障碍感知更新
                    if args.use_apf and args.use_obstacle:
                        renderer_depth.update_scene(data, camera="rear_cam")
                        depth = renderer_depth.render()
                        pcd = depth_to_point_cloud(model, data, depth, "rear_cam", intrinsics)
                        cap = extract_obstacle_capsule(pcd)
                        if cap is not None:
                            if last_obs is None:
                                last_obs = cap
                            else:
                                d = np.linalg.norm(cap["p1"][:2] - last_obs["p1"][:2])
                                if d < 0.04:
                                    a = 0.8
                                    last_obs["p1"] = a*last_obs["p1"] + (1-a)*cap["p1"]
                                    last_obs["p2"] = a*last_obs["p2"] + (1-a)*cap["p2"]
                                    last_obs["r"] = a*last_obs["r"] + (1-a)*cap["r"]

                    # 轨迹预判与全局替换
                    if args.use_apf and args.use_obstacle and last_obs is not None:
                        collide, idx = check_trajectory_safety(model, data, cache, last_obs, link6_id, tcp_id)
                        if collide:
                            replace_count += 1
                            screw_pos = data.xpos[target_body_id]
                            tcp_pos = data.site_xpos[tcp_id]
                            box_pos = np.array([0.0, -0.1, 0.24])
                            is_grasped = (data.ctrl[6] < 0.02 and np.linalg.norm(tcp_pos - screw_pos) < 0.05)
                            goal = screw_pos if not is_grasped else box_pos
                            traj = generate_apf_global_trajectory(model, data, goal, last_obs, link6_id, tcp_id)
                            q_traj = [solve_ik(model, data, p, tcp_id) for p in traj]
                            smooth = smooth_trajectory(data.qpos[:6], q_traj)
                            for i in range(8):
                                safe_cache[i][:6] = smooth[i]

                # 视频帧记录
                renderer_rgb.update_scene(data, camera="rear_cam")
                frames.append(renderer_rgb.render())
                act = safe_cache[step % 8]

                # 兜底逐帧APF逻辑
                if args.use_apf and not args.use_obstacle:
                    ctrl, _, _ = process_apf_action(model, data, act, last_obs, step)
                    data.ctrl[:] = ctrl
                else:
                    # 【核心优化3】关节速度限制从±0.25→±0.4，关节转动速度翻倍
                    dq = act[:6] - data.qpos[:6]
                    data.ctrl[:6] = data.qpos[:6] + np.clip(dq, -0.4, 0.4)
                    data.ctrl[6:8] = 0.04 if act[6] > 0.02 else 0.0

                # 障碍可视化
                if args.use_obstacle and last_obs is not None:
                    viewer.user_scn.ngeom = 0
                    g = viewer.user_scn.geoms[0]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), [1,1,0,0.3])
                    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, last_obs["r"], last_obs["p1"], last_obs["p2"])
                    viewer.user_scn.ngeom = 1

                # 【核心优化4】单次仿真步数从50→25，动作更新更频繁，视觉更流畅
                for _ in range(25):
                    mujoco.mj_step(model, data)

                # 碰撞检测
                if args.use_obstacle and pillar_body_id != -1:
                    if data.xmat[pillar_body_id].reshape(3,3)[2,2] < 0.9:
                        col = True

                # 【核心优化5】任务完成判定从5帧→10帧，避免误判提前结束
                s_pos = data.xpos[target_body_id]
                in_box_flag = abs(s_pos[0])<0.06 and abs(s_pos[1]+0.1)<0.06 and s_pos[2]<0.34
                released = act[6]>0.02 or np.linalg.norm(data.site_xpos[tcp_id]-s_pos) > 0.06
                if in_box_flag and released:
                    in_box +=1
                else:
                    in_box =0
                if in_box >= 10:
                    ok = True
                    break

                viewer.sync()
                step +=1

            # 回合统计
            if ok: success +=1
            if col: collision +=1
            smoothness = 0
            if len(qhist) >2:
                smoothness = np.mean(np.linalg.norm(np.diff(np.diff(qhist, axis=0), axis=0), axis=1))
            folder = os.path.join("recordings", "success" if ok else "fail")
            save_episode_video(frames, folder, ep+1)
            print(f"EP {ep+1:02d} | 成功:{ok} | 碰撞:{col} | 轨迹替换:{replace_count}次 | 成功率:{success/(ep+1)*100:.1f}% | 平滑度:{smoothness:.3f} | 总步数:{step}")

    glfw.terminate()

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--use_obstacle", action="store_true")
    p.add_argument("--use_apf", action="store_true")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=8000)
    main(p.parse_args())