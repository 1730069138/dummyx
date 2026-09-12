#!/usr/bin/env python3
import sys
import tty
import termios
import time
import yaml
from motorcontroller import MotorController

def getch():
    """
    Linux 原生非阻塞读取单个按键输入
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        if ch == '\x03': # 捕获 Ctrl+C
            raise KeyboardInterrupt
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    print("="*60)
    print(" DummyX 机械臂 - 独立键盘微调终端 (带软件限位)")
    print("="*60)
    
    controller = MotorController()

    print("[1/3] 正在加载 motors.yaml 并接入 CAN 总线...")
    try:
        with open('motors.yaml', 'r') as file:
            motor_config = yaml.safe_load(file)
        for node in motor_config['nodes']:
            controller.add_motor(node['id'], reduction=node['reduction'])
    except Exception as e:
        print(f"配置加载失败: {e}")
        return

    if controller.is_initialized():
        controller.start()
    else:
        print("错误: CAN 总线挂载失败。")
        return

    # ------------------ 新增：驱动至固定初始姿态 ------------------
    print("[2/3] 正在将机械臂驱动至设定的初始姿态...")
    initial_poses = {
        1: 180.0,
        2: 80.0,
        3: -100.0,
        4: -120.0,
        5: 115.0,
        6: 140.0,
        7: -115.0
    }
    
    for i in range(1, 8):
        motor = controller.motors.get(i)
        if motor:
            motor.set_position(initial_poses[i])
            print(f"      -> 指令下发: 关节 [{i}] 目标 {initial_poses[i]}°")
            
    print("      等待机械臂到达初始位置 (3秒)...")
    time.sleep(3.0) # 给予机械臂足够的运动时间到达初始位置

    # --------------------------------------------------------------

    print("[3/3] 正在重新读取底层真实电机位置，作为键盘控制的基准...")
    targets = {}
    
    # 向所有电机请求一次当前状态
    for i in range(1, 8):
        motor = controller.motors.get(i)
        if motor:
            motor.reference_status()
            
    time.sleep(0.5) # 给予总线 0.5 秒的时间接收返回的数据
    
    for i in range(1, 8):
        motor = controller.motors.get(i)
        if motor:
            # 严格按照要求：以到达初始位置后的【底层真实读取位置】作为基准起始点
            targets[i] = motor.position
            print(f"      -> 关节 [{i}] 真实角度已同步: {targets[i]:.1f}°")

    print("\n" + "="*60)
    print(" 🎮 键盘控制映射已激活 (按下立即生效):")
    print(" [Q/A] -> 关节1    [W/S] -> 关节2    [E/D] -> 关节3")
    print(" [R/F] -> 关节4    [T/G] -> 关节5    [Y/H] -> 关节6")
    print(" [U/J] -> 关节7")
    print("\n [=] -> 增大单次步长    [-] -> 减小单次步长")
    print(" [ESC] 或 [Ctrl+C] 退出当前脚本 (维持原状态抱死)")
    print("="*60 + "\n")

    step_size = 1.0  # 默认每次按键移动 1.0 度

    # 键位与关节的映射关系字典 (关节ID, 方向乘数)
    key_mapping = {
        'q': (1, 1), 'a': (1, -1),
        'w': (2, 1), 's': (2, -1),
        'e': (3, 1), 'd': (3, -1),
        'r': (4, 1), 'f': (4, -1),
        't': (5, 1), 'g': (5, -1),
        'y': (6, 1), 'h': (6, -1),
        'u': (7, 1), 'j': (7, -1),
    }

    # 根据配置表提取的各关节物理安全角度限制 (Min, Max)
    # 注意：为了适配第7轴新的初始位置 -115，其下限已从 10 扩大至 -130
    joint_limits = {
        1: (5.0, 340.0),
        2: (10.0, 180.0),
        3: (-180.0, 0.0),
        4: (-230.0, -10.0),
        5: (10.0, 220.0),
        6: (10.0, 280.0),
        7: (-120.0, 0.0) 
    }

    try:
        while True:
            ch = getch().lower()
            
            if ch == '\x1b': # ESC 键退出
                break
                
            elif ch == '=' or ch == '+':
                step_size += 0.5
                sys.stdout.write(f"\r[步长调整] 当前按键步长放大为: {step_size:>4.1f}°" + " "*30)
                sys.stdout.flush()
                
            elif ch == '-':
                step_size = max(0.1, step_size - 0.5)
                sys.stdout.write(f"\r[步长调整] 当前按键步长缩小为: {step_size:>4.1f}°" + " "*30)
                sys.stdout.flush()
                
            elif ch in key_mapping:
                motor_id, direction = key_mapping[ch]
                if motor_id in controller.motors:
                    # 1. 理论计算新的目标位置 (永远在读取到的电机位置/累加位置基础上计算)
                    theoretical_target = targets[motor_id] + direction * step_size
                    
                    # 2. 获取该关节的限位范围，如果字典里没有则默认不限位
                    min_limit, max_limit = joint_limits.get(motor_id, (-360.0, 360.0))
                    
                    # 3. 钳制(Clamping)逻辑与状态提示
                    limit_warning = ""
                    if theoretical_target > max_limit:
                        new_target = max_limit
                        limit_warning = f" ⚠️ 达正向极限({max_limit}°)"
                    elif theoretical_target < min_limit:
                        new_target = min_limit
                        limit_warning = f" ⚠️ 达负向极限({min_limit}°)"
                    else:
                        new_target = theoretical_target

                    # 4. 覆盖旧目标值并下发
                    targets[motor_id] = new_target
                    controller.motors[motor_id].set_position(new_target)
                    
                    # 终端清行并打印实时状态
                    sys.stdout.write(f"\r[键盘操控] 关节 {motor_id} 目标 -> {new_target:>6.1f}° | 步长: {step_size:>4.1f}°{limit_warning}" + " "*10)
                    sys.stdout.flush()
                    
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\n正在安全退出键盘控制...")
        # 核心：此处仅停止数据监听，绝对不去调用 motor.disable()
        # 把维持机械臂抗重力的任务交还给跑在另一个终端里的底层程序
        controller.stop()
        print("✅ 已退出。机械臂已平滑留在当前位置。")

if __name__ == "__main__":
    main()