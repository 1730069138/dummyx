from core.paths import MOTORS_CONFIG, URDF_PATH, load_mujoco_model
import cv2
import numpy as np
import pyrealsense2 as rs
import mujoco
import time
import yaml
from core.motorcontroller import MotorController

# ==========================================
# 1. 填入刚才算出来的标定矩阵！
# ==========================================
T_cam2base = np.array([
    [ 0.9256,  0.1621,  0.3419, -0.1631],
    [-0.1413,  0.9863, -0.0850,  0.0375],
    [-0.3510,  0.0304,  0.9359, -0.2287],
    [ 0.0000,  0.0000,  0.0000,  1.0000]
])

# 计算 base -> cam 的逆矩阵 (用于把机械臂坐标转到相机画面)
T_base2cam = np.linalg.inv(T_cam2base)

# ==========================================
# 2. 初始化硬件与引擎
# ==========================================

model = load_mujoco_model()
data = mujoco.MjData(model)
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6") # 依附于上一级的 link6

print("正在启动 D415 相机...")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
profile = pipeline.start(config)

intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                          [0, intrinsics.fy, intrinsics.ppy],
                          [0, 0, 1]])
dist_coeffs = np.zeros(5)

controller = MotorController(interface='socketcan', channel='can0')
controller.start()
with open(MOTORS_CONFIG, 'r') as f:
    for node in yaml.safe_load(f)['nodes']:
        controller.add_motor(node['id'], reduction=node['reduction'])
time.sleep(1)

JOINT_OFFSETS = [3.094, 1.396, 0.0, -2.269, 1.920, 2.287]

# ==========================================
# 3. 实时 AR 投影验证
# ==========================================
print("\n✅ AR 验证启动！请随意移动机械臂，观察画面中的坐标轴是否贴合夹爪。")
try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue
        img = np.asanyarray(color_frame.get_data())
        
        # 1. 读电机并解算 URDF 位姿
        real_qpos = []
        for i in range(1, 7):
            controller.motors[i].reference_value1()
            time.sleep(0.005) # 加快刷新率
            real_qpos.append(np.deg2rad(controller.motors[i].position) - JOINT_OFFSETS[i-1])
            
        data.qpos[:6] = real_qpos
        mujoco.mj_kinematics(model, data)
        
        # 提取夹爪在 Base 坐标系下的位姿
        T_gripper2base = np.eye(4)
        T_gripper2base[:3, :3] = data.xmat[ee_id].reshape(3, 3)
        T_gripper2base[:3, 3] = data.xpos[ee_id]
        
        # 2. 【核心魔法】将夹爪坐标系映射到相机坐标系
        T_gripper2cam = T_base2cam @ T_gripper2base
        R_gripper2cam = T_gripper2cam[:3, :3]
        t_gripper2cam = T_gripper2cam[:3, 3]
        
        # 转换为 OpenCV 需要的旋转向量
        rvec, _ = cv2.Rodrigues(R_gripper2cam)
        
        # 3. 在画面上画出 10厘米 长的 XYZ 坐标轴
        cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, t_gripper2cam, 0.1)
        
        cv2.imshow("AR Verification", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    controller.stop()
