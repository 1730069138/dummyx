import cv2
import numpy as np
import pyrealsense2 as rs
import mujoco
import time
import yaml
import os
from motorcontroller import MotorController

# ==========================================
# 1. 核心参数设置
# ==========================================
# 【请核对】标定板二维码黑色方块的精确边长（单位：米）
MARKER_SIZE = 0.09669 
# URDF 绝对路径
URDF_PATH = "/home/jun/dummyx/webgui/dummy_real_v3/urdf/dummy_real_v3.urdf"

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

R_gripper2base, t_gripper2base = [], []
R_target2cam, t_target2cam = [], []

# ==========================================
# 2. MuJoCo 运动学引擎 (使用绝对路径加载)
# ==========================================
print(f"正在从绝对路径解析 URDF: {URDF_PATH}")
if not os.path.exists(URDF_PATH):
    print(f"❌ 错误：在路径 {URDF_PATH} 下找不到 URDF 文件！")
    exit()

try:
    model = mujoco.MjModel.from_xml_path(URDF_PATH)
    data = mujoco.MjData(model)
except Exception as e:
    print(f"❌ MuJoCo 加载 URDF 失败: {e}")
    exit()

# 末端连杆设定
END_EFFECTOR_NAME = "link6"
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, END_EFFECTOR_NAME)
if ee_id == -1:
    print(f"⚠️ 致命错误：在 URDF 中找不到 body '{END_EFFECTOR_NAME}'！")
    exit()

# ==========================================
# 3. 初始化硬件 (D415 相机 & 机械臂)
# ==========================================
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

print("正在连接实体机械臂并读取 motors.yaml...")
controller = MotorController(interface='socketcan', channel='can0')
controller.start()

try:
    # 假设 motors.yaml 在当前运行目录下
    with open('motors.yaml', 'r') as f:
        motor_config = yaml.safe_load(f)
    for node in motor_config['nodes']:
        controller.add_motor(node['id'], reduction=node['reduction'])
    print("✅ 电机配置加载成功！")
except Exception as e:
    print(f"❌ 读取 motors.yaml 失败: {e}")
    exit()
time.sleep(1)

# ==========================================
# 4. 采集循环
# ==========================================
print("\n✅ 标定系统就绪（已使用绝对路径 URDF）")
print("操作：按 'c' 记录当前姿态，按 'q' 退出并解算。")

# 关节偏置矫正参数（弧度）
# 逻辑：URDF 角度 = 电机原始弧度 - 偏置
JOINT_OFFSETS = [3.072, 1.396, 0.0, 0.0, 0.0, 0.0]

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue
        
        img = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_matrix, dist_coeffs)
            cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.05)
            
        cv2.imshow("Calibration Window", img)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('c') and ids is not None:
            # 记录视觉位姿
            R_cam, _ = cv2.Rodrigues(rvecs[0])
            R_target2cam.append(R_cam)
            t_target2cam.append(tvecs[0].reshape(3, 1))
            
            # 记录并修正关节角度
            real_qpos = []
            for i in range(1, 7):
                controller.motors[i].reference_value1()
                time.sleep(0.01)
                
                raw_rad = np.deg2rad(controller.motors[i].position)
                # 逆向应用偏置，对齐 URDF 坐标系
                corrected_rad = raw_rad - JOINT_OFFSETS[i-1]
                real_qpos.append(corrected_rad)
                
            # 更新 MuJoCo 运动学
            data.qpos[:6] = real_qpos
            mujoco.mj_kinematics(model, data) 
            
            # 提取 link7 位姿
            R_base = data.xmat[ee_id].copy().reshape(3, 3)
            t_base = data.xpos[ee_id].copy().reshape(3, 1)
            
            R_gripper2base.append(R_base)
            t_gripper2base.append(t_base)
            
            print(f"📸 成功记录第 {len(R_gripper2base)} 组有效数据")
            
        elif key == ord('q'):
            break
            
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    controller.stop()

# ==========================================
# 5. 计算结果
# ==========================================
if len(R_gripper2base) < 10:
    print("\n⚠️ 采集样本不足（建议至少 15 组），解算可能不准。")
    
if len(R_gripper2base) > 0:
    print("\n⏳ 正在进行 Tsai-Lenz 手眼标定解算...")
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base, 
        R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    T_cam2base = np.eye(4)
    T_cam2base[:3, :3] = R_cam2base
    T_cam2base[:3, 3] = t_cam2base.flatten()
    
    print("\n🎉 标定完成！得到的相机到底座变换矩阵 (T_cam2base):")
    print("--------------------------------------------------")
    print(np.array_str(T_cam2base, precision=4, suppress_small=True))
    print("--------------------------------------------------")