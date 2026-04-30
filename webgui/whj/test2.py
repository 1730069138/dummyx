#!/usr/bin/env python3
"""
功能：交互式定点控制。通过按键（1-6）控制机械臂前往指定点位。
"""

import os
import sys
import time
import threading
import subprocess
import yaml

# 添加当前目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    from motorcontroller import MotorController
except ImportError as e:
    print(f"导入错误: {e}")
    sys.exit(1)

# ==========================================
# 1. 姿态配置区域 (点位与键位映射)
# ==========================================
def create_pose(j1, j2, j3, j4, j5, j6):
    return {1: j1, 2: j2, 3: j3, 4: j4, 5: j5, 6: j6}

# 定义你的点位字典
POSES = {
    '1': create_pose(187.0, 111.0, -82.9, -129.0, 180.0, 130.0),
    '2': create_pose(180.0, 111.0, -82.9, -129.0, 180.0, 130.0),
    '3': create_pose(172.0, 111.0, -82.9, -129.0, 180.0, 130.0),
    '4': create_pose(172.0, 106.0, -91.9, -129.0, 180.0, 130.0),
    '5': create_pose(180.0,106.0, -91.9, -129.0, 180.0, 130.0),
    '6': create_pose(189.0,106.0, -90.9, -129.0, 180.0, 130.0),
    'h': create_pose(180.001, 80.001, -85.995,-130.00,150.001,130.0), # Home位
}

# ==========================================
# 2. 运行参数
# ==========================================
POSITION_TOLERANCE = 0.5
MOVE_TIMEOUT = 5.0

controller = None
stop_event = threading.Event()

def setup_can():
    """配置 CAN-FD"""
    print(">>> 配置 CAN-FD 总线...")
    try:
        subprocess.run("sudo ip link set can0 down", shell=True)
        subprocess.run("sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on", shell=True, check=True)
        subprocess.run("sudo ip link set can0 up", shell=True, check=True)
        return True
    except:
        return False

def init_system():
    """初始化"""
    global controller
    try:
        setup_can()
        controller = MotorController(interface='socketcan', channel='can0')
        config_path = os.path.join(current_dir, 'motors.yaml')
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        for node in cfg['nodes']:
            controller.add_motor(node['id'], reduction=node['reduction'])
        controller.start()
        time.sleep(0.5)
        for motor in controller.motors.values():
            motor.error_resets()
            time.sleep(0.05)
            motor.enable()
        return True
    except Exception as e:
        print(f"初始化失败: {e}")
        return False

def move_to_pose(pose_dict, label=""):
    """执行移动并等待到达"""
    print(f"\n[运动] 目标点: {label}")
    
    # 下发目标位置
    for j_id, angle in pose_dict.items():
        if j_id in controller.motors:
            controller.motors[j_id].set_position(angle)
    
    # 监控是否到达
    start_time = time.time()
    while (time.time() - start_time) < MOVE_TIMEOUT:
        all_ok = True
        for j_id, target in pose_dict.items():
            motor = controller.motors[j_id]
            motor.reference_value1() # 更新位置读取
            if abs(motor.position - target) > POSITION_TOLERANCE:
                all_ok = False
        
        if all_ok:
            print(f"  [就绪] 已安全到达 {label}")
            return True
        time.sleep(0.05)
        
    print(f"  [超时] 未能完全到达目标位置")
    return False

def main():
    if not init_system():
        return

    print("\n" + "="*40)
    print("   机械臂交互控制系统")
    print("   1-6 : 前往对应点位")
    print("   h   : 回到 Home 位置")
    print("   q   : 停止并退出")
    print("="*40)

    try:
        while True:
            # 获取用户输入
            key = input("\n请输入指令 >> ").strip().lower()

            if key == 'q':
                print(">>> 正在退出...")
                break
            
            if key in POSES:
                label = f"点位 {key}" if key != 'h' else "HOME"
                move_to_pose(POSES[key], label)
                
                # 到达后的状态显示
                positions = []
                for mid in sorted(controller.motors.keys()):
                    positions.append(f"J{mid}:{controller.motors[mid].position:.1f}")
                print(f"当前位置: {' | '.join(positions)}")
            else:
                print("无效输入！请输入 1-6, h 或 q。")

    except KeyboardInterrupt:
        pass
    finally:
        print("\n>>> 安全清理中...")
        # 退出前尝试回位（可选）
        move_to_pose(POSES['h'], "HOME安全位置")
        if controller:
            for m in controller.motors.values():
                m.disable()
            controller.stop()
        print("程序结束")

if __name__ == "__main__":
    main()

