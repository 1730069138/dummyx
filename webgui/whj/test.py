#!/usr/bin/env python3
"""
功能：在平面内循环运行 9 个预设姿态点
优化版：模块化、易扩展、增加安全保护
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
# 1. 姿态配置区域 (在此修改你的 9 个点)
# ==========================================
def create_pose(j1, j2, j3, j4, j5, j6):
    """辅助函数：快速创建关节字典"""
    return {1: j1, 2: j2, 3: j3, 4: j4, 5: j5, 6: j6}

# 初始/安全位置
# POSE_HOME = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# 定义平面上的 9 个点 (你可以根据实际平面坐标计算出的逆解角度填入)
POINTS_ON_PLANE = [
    create_pose(187.0, 111.0, -82.9, -129.0, 180.0, 130.0),  # 点 1
    create_pose(180.0, 111.0, -82.9, -129.0, 180.0, 130.0),  # 点 2
    create_pose(172.0, 111.0, -82.9, -129.0, 180.0, 130.0),  # 点 3
    create_pose(172.0, 106.0, -91.9, -129.0, 180.0, 130.0),  # 点 4
    create_pose(180.0,106.0, -91.9, -129.0, 180.0, 130.0),  # 点 5
    create_pose(189.0,106.0, -90.9, -129.0, 180.0, 130.0),# 点 6
]

# ==========================================
# 2. 运行参数配置
# ==========================================
LOOP_CYCLES = 1       # 循环次数（0=无限循环）
STAY_TIME = 1.5       # 在每个点停留的时间（秒）
POSITION_TOLERANCE = 0.5  # 容差角度（度），越小越精确但耗时更长
MOVE_TIMEOUT = 8.0    # 单次移动最大允许时间

# 全局状态控制
controller = None
running = False
stop_event = threading.Event()

def setup_can():
    """配置 CAN 总线 (略，保持原逻辑)"""
    print(">>> 配置 CAN-FD 总线...")
    # ... (保持原有的 subprocess 代码)
    commands = [
        "sudo ip link set can0 down",
        "sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on",
        "sudo ip link set can0 up"
    ]
    for cmd in commands:
        subprocess.run(cmd, shell=True, check=True, capture_output=True)
    time.sleep(1)
    return True

def init_system():
    """系统初始化：CAN、控制器、电机使能"""
    global controller
    try:
        # 1. CAN
        setup_can()
        # 2. 控制器
        controller = MotorController(interface='socketcan', channel='can0')
        config_path = os.path.join(current_dir, 'motors.yaml')
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        
        for node in cfg['nodes']:
            controller.add_motor(node['id'], reduction=node['reduction'])
        
        controller.start()
        time.sleep(0.5)
        
        # 3. 电机使能与清除错误
        print(">>> 正在准备电机...")
        for motor in controller.motors.values():
            motor.error_resets()
            time.sleep(0.05)
            motor.enable()
        
        print(">>> 所有电机已就绪")
        return True
    except Exception as e:
        print(f"初始化失败: {e}")
        return False

def move_to_pose(pose_dict, label=""):
    """
    通用移动函数
    :param pose_dict: 目标位置字典
    :param label: 当前点位的描述信息
    """
    if stop_event.is_set(): return False
    
    print(f"\n[运动] 目标: {label}")
    
    # 下发指令
    for j_id, angle in pose_dict.items():
        if j_id in controller.motors:
            controller.motors[j_id].set_position(angle)
    
    # 闭环监控
    start_time = time.time()
    while (time.time() - start_time) < MOVE_TIMEOUT:
        if stop_event.is_set(): return False
        
        all_ok = True
        status_str = []
        
        for j_id, target in pose_dict.items():
            motor = controller.motors[j_id]
            motor.reference_value1() # 更新位置
            time.sleep(0.01)
            diff = abs(motor.position - target)
            if diff > POSITION_TOLERANCE:
                all_ok = False
            status_str.append(f"J{j_id}:{motor.position:>.1f}")
            
        if all_ok:
            print(f"  [到达] {label} 已就位")
            return True
        
        # 打印实时进度
        print(f"\r  进度: {' | '.join(status_str)}", end="")
        time.sleep(0.1)
        
    print(f"\n  [警告] {label} 移动超时！")
    return False

def run_mission():
    """主任务循环"""
    global running
    running = True
    cycle_count = 0
    
    try:
        while not stop_event.is_set():
            if LOOP_CYCLES > 0 and cycle_count >= LOOP_CYCLES:
                break
                
            cycle_count += 1
            print(f"\n{'#'*60}\n# 开始第 {cycle_count} 轮巡航\n{'#'*60}")
            
            # 依次遍历 6 个点
            for i, pose in enumerate(POINTS_ON_PLANE):
                if stop_event.is_set(): break
                
                # 执行移动
                success = move_to_pose(pose, label=f"点位-{i+1}")
                
                if success:
                    # 到达后停留，带倒计时显示
                    t_start = time.time()
                    while time.time() - t_start < STAY_TIME and not stop_event.is_set():
                        rem = STAY_TIME - (time.time() - t_start)
                        print(f"\r  [等待] 停留中... 剩余 {rem:.1f}s", end="")
                        time.sleep(0.1)
                    print()
            
            print(f"\n>>> 第 {cycle_count} 轮完成")

    finally:
        # 无论成功失败，最后尝试安全回位
        print("\n>>> 任务结束，正在回位...")
        # move_to_pose(POSE_HOME, "HOME安全位置")
        running = False

def main():
    global running
    print("=== 六点平面运动演示程序 ===")
    
    if not init_system():
        return

    try:
        # 显示当前位置
        print("\n当前关节角度:")
        for mid, m in controller.motors.items():
            m.reference_value1()
            time.sleep(0.02)
            print(f"  J{mid}: {m.position:.1f}")

        confirm = input("\n[确认] 确保周围安全，按 'y' 开始运行: ")
        if confirm.lower() != 'y':
            return

        # 启动任务线程
        task_thread = threading.Thread(target=run_mission, daemon=True)
        task_thread.start()

        # 主线程等待中断
        while running:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[紧急停止] 接收到用户中断指令")
        stop_event.set()
    finally:
        while running: time.sleep(0.1)
        print(">>> 正在释放电机并退出...")
        # if controller:
        #     for m in controller.motors.values():
        #         m.disable()
        #     controller.stop()
        print("程序已安全关闭")

if __name__ == "__main__":
    main()

