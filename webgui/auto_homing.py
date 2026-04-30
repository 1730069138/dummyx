import time
import threading
import yaml
import sys
import struct
import can
from motorcontroller import MotorController

def hardware_polling_loop(controller):
    """
    后台数据泵：稳健获取实时的位置、速度和电流，绝不拥堵总线
    """
    while True:
        for motor in list(controller.motors.values()):
            try:
                motor.reference_status()
                time.sleep(0.01)
                motor.reference_value1()
                time.sleep(0.01)
            except Exception:
                pass
        time.sleep(0.05)

def smart_homing(motor, node_id, homing_current, homing_pos, expected_pos):
    """
    智能调零序列：彻底绕开底层同值死锁 Bug，采用纯软件憋停检测
    """
    print(f"\n[{node_id}] ----------------------------------------")
    print(f"[{node_id}] 1. 清洗底层状态机与错误锁死...")
    motor.status.over_current = False
    motor.error_resets()
    time.sleep(0.2)
    motor.disable()
    time.sleep(0.2)

    print(f"[{node_id}] 2. 注入防死锁参数 (运行={homing_current}A, 保护=20A)...")
    tx_id = motor.build_can_id(dir_bit=0, cmd_id=17) # CMD_SET_CONFIG = 0x11
    
    # 设定运行限流
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 7, float(homing_current)), is_extended_id=False))
    time.sleep(0.05)
    # 把硬件过流保护阈值撑大到20A，绝不允许底层硬件随意切断动力
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 28, 20.0), is_extended_id=False))
    time.sleep(0.05)

    # 设定搜零运动速度 (Vel=5, Acc=10, Dec=10)
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 23, 5.0), is_extended_id=False))
    time.sleep(0.05)
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 24, 10.0), is_extended_id=False))
    time.sleep(0.05)
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 25, 10.0), is_extended_id=False))
    time.sleep(0.05)

    print(f"[{node_id}] 3. 强行切换至位置模式并使能...")
    motor.set_stop_damping_mode()
    time.sleep(0.2)
    motor.enable()
    time.sleep(0.3)

    print(f"[{node_id}] 4. 下发探零运动指令: {homing_pos}°")
    start_pos = motor.position
    motor.set_position(homing_pos)
    
    start_t = time.time()
    stall_counter = 0
    has_backed_off = False  # 标记是否已经触发过反向回退
    
    # 核心：防死锁实时监测循环
    while True:
        elapsed = time.time() - start_t
        if elapsed > 40:
            print(f"\n[{node_id}] ❌ 调零超时！(可能原因: 电机被卡死未移动，或 {homing_current}A 无法克服静摩擦)")
            return "HOMING_FAIL"
        
        vel = abs(motor.motor_velocity)
        pos = motor.position
        curr = abs(motor.motor_current)  # 获取实时电流
        
        # 动态回车打印，实时监控运动情况 (增加了电流显示)
        sys.stdout.write(f"\r[{node_id}] 实时状态 -> 位置: {pos:6.1f}°, 速度: {vel:5.1f}, 电流: {curr:5.2f}A")
        sys.stdout.flush()
        
        # 兜底：如果检测到了正常的硬件过流 (已经跑出一段距离了)
        if motor.status.over_current:
            if abs(pos - start_pos) >= 2.0 or has_backed_off:
                print(f"\n[{node_id}] ⚠️ 触碰限位触发极值过流，视作调零成功！")
                break
                
        # --- 精准的起步卡死防呆与相对反向解锁逻辑 ---
        # 如果下发指令超过 2秒没动，或者刚起步就因为推墙引发过流，判定为贴墙
        if not has_backed_off and (elapsed > 2.0 or motor.status.over_current):
            if abs(pos - start_pos) < 2.0:
                print(f"\n[{node_id}] ⚠️ 发现起步即卡死(可能已在限位)，执行相对反向解锁...")
                
                motor.status.over_current = False
                motor.disable()
                time.sleep(0.1)
                motor.error_resets()
                time.sleep(0.1)
                motor.set_stop_damping_mode()
                time.sleep(0.1)
                motor.enable()
                time.sleep(0.2)
                
                relative_offset = 30.0 if homing_pos < 0 else -30.0
                backoff_target = pos + relative_offset
                print(f"[{node_id}] 🔙 正在下发相对退让指令: 当前 {pos:.1f}° -> 去往 {backoff_target:.1f}°")
                motor.set_position(backoff_target)
                
                time.sleep(3.0)
                
                start_pos = motor.position
                print(f"[{node_id}] 🔄 退让完成，重新下发原探零指令: {homing_pos}°")
                motor.set_position(homing_pos)
                
                start_t = time.time()
                has_backed_off = True
                stall_counter = 0
                continue
        # ---------------------------------------------

        # 屏蔽起步前 1.5 秒的干扰，专注于正常的寻零憋停检测
        if elapsed > 1.5:
            # 【核心判定】：速度极低 + 离开原点 + 电流顶到了设定限制的 60% 以上
            if vel < 0.4 and abs(pos - start_pos) > 2.0 and curr > (homing_current * 0.6):
                stall_counter += 1
            else:
                stall_counter = 0
                
            if stall_counter >= 15:
                print(f"\n[{node_id}] ✅ 成功检测到物理限位停转 (持续1.5秒高电流憋停)！")
                break
                
        time.sleep(0.1)
        
    print(f"[{node_id}] 5. 正在将当前碰撞点强制写为绝对零点...")
    motor.error_resets()
    time.sleep(0.1)
    motor.disable()
    time.sleep(0.2)
    motor.set_home()
    time.sleep(0.5)
    
    # ================== 核心修正区：恢复输出力气与平缓回退 ==================
    # 1. 恢复原厂过流保护值，保障后续安全
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 28, 12.0), is_extended_id=False))
    time.sleep(0.05)
    # 2. 强行恢复电机的运行限流 (10.0A)，给电机充足的力矩抵抗前端重力，防止掉落
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 7, 10.0), is_extended_id=False))
    time.sleep(0.05)
    # 3. 将回退动作的速度和加速度压低，让回退更加极度平滑
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 23, 2.0), is_extended_id=False)) # 回退速度限制为2
    time.sleep(0.05)
    motor.bus.send(can.Message(arbitration_id=tx_id, data=struct.pack("<If", 24, 5.0), is_extended_id=False)) # 回退加速度限制为5
    time.sleep(0.05)
    # ========================================================================
    
    motor.enable()
    time.sleep(0.2)
    
    print(f"[{node_id}] 6. 正在优雅回退到安全姿态: {expected_pos}°")
    motor.set_position(expected_pos)
    
    # 动态监测回退是否到位
    reach_timeout = 30.0  
    start_backoff_t = time.time()
    reached = False
    
    while True:
        elapsed_backoff = time.time() - start_backoff_t
        if elapsed_backoff > reach_timeout:
            break
            
        current_pos = motor.position
        diff = abs(current_pos - expected_pos)
        
        sys.stdout.write(f"\r[{node_id}] 回退监控 -> 当前位置: {current_pos:6.1f}°, 目标: {expected_pos:6.1f}° (偏差: {diff:5.1f}°)")
        sys.stdout.flush()
        
        if diff < 1.5:
            reached = True
            break
            
        time.sleep(0.1)
        
    print() # 换行
    if reached:
        print(f"[{node_id}] 🎉 关节已精准到达安全姿态，调零圆满完成。")
        return True
    else:
        print(f"[{node_id}] ❌ 致命错误：回退动作超时(30s)！电机卡在 {motor.position:.1f}°，未能到达安全姿态。")
        return "BACKOFF_FAIL"

def main():
    print("="*60)
    print(" DummyX 智能透明调零脚本 (第五关节首发调零+跳过逻辑版)")
    print("="*60)
    
    controller = MotorController()

    print("[1/5] 正在加载 motors.yaml 并启动 CAN 通信...")
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
        print("错误: CAN 总线初始化失败。")
        return

    print("[2/5] 正在启动后台数据泵...")
    threading.Thread(target=hardware_polling_loop, args=(controller,), daemon=True).start()
    time.sleep(1.5) 

    # 提取专属于 DummyX 的参数 (限制电流, 碰撞方向角度, 回退安全角度)
    homing_params = {
        1: (3.0, -360, 180),
        2: (4.0, -360, 81),
        3: (4.0, 360, -85),
        4: (2.0, 360, -130),
        5: (2.0, -360, 110),
        6: (1.0, -180, 130), # <- 严格保留：第六关节碰撞调零后转到 130 度
        7: (1.0, -120, 0)    # <- 第七关节的寻零配置
    }
    
    final_target_positions = {
        1: 180.001,
        2: 80.001,
        3: -85.995,
        4: -130.001,
        5: 150.001,
        6: 130.0,    # <- 严格保留：第六关节最终归位目标保持在 130 度
        7: 50.001     # <- 第七关节最终归位目标
    }

    # ================= 新增：一开始只使能并调零第五关节 =================
    print("\n[3/5] 预动作：仅使能第五关节进行寻零，并停留在零位...")
    motor5 = controller.motors.get(5)
    if motor5:
        params5 = homing_params.get(5)
        if params5:
            # 注意这里将 expected_pos 参数强制传入为 0.0，让它完成寻零后保持在限位不动
            result = smart_homing(motor5, 5, params5[0], params5[1], 0.0)
            
            if result == "BACKOFF_FAIL":
                time.sleep(2)
                result = smart_homing(motor5, 5, params5[0], params5[1], 0.0)
                
            if result != True:
                print("\n⚠️ 第五关节预先调零失败，已安全终止。")
                sys.exit(1)
    # ===================================================================

    print("\n[4/5] 严格按序执行智能碰撞调零 (第4个结束后第5个直接移动)...")
    # 包含 1 到 7 轴
    for i in range(1, 8):
        motor = controller.motors.get(i)
        if not motor:
            continue
            
        # ================= 核心：对第五关节跳过调零，直接移动 =================
        if i == 5:
            target_pos = final_target_positions.get(5, 135.001)
            print(f"\n---> [5] 第四关节已结束，第五关节无需调零，直接移动至待机位置: {target_pos}° ...")
            motor.set_position(target_pos)
            
            reach_timeout = 40.0
            start_t = time.time()
            reached = False
            while True:
                elapsed = time.time() - start_t
                if elapsed > reach_timeout:
                    break
                curr_pos = motor.position
                diff = abs(curr_pos - target_pos)
                sys.stdout.write(f"\r     Motor [{i}] 移动监控 -> 当前: {curr_pos:6.1f}°, 目标: {target_pos:6.1f}° (偏差: {diff:5.1f}°)")
                sys.stdout.flush()
                if diff < 1.5:
                    reached = True
                    break
                time.sleep(0.1)
            print() 
            if not reached:
                print(f"     ⚠️ 警告: Motor [{i}] 移动超时，未能精准到达最终位置！(停在 {motor.position:.1f}°)")
                sys.exit(1) 
            else:
                print(f"     ✅ Motor [{i}] 已安全就位。")
            continue # 完成第5关节直接移动，跳过下方常规调零逻辑，进入第6关节
        # =====================================================================

        params = homing_params.get(i)
        if params:
            # 第一次尝试调零
            result = smart_homing(motor, i, params[0], params[1], params[2])
            
            # --- 新增：回退失败的重试容错机制 ---
            if result == "BACKOFF_FAIL":
                print(f"\n⚠️ 触发容错机制：Motor [{i}] 回退不到位，可能因偶发摩擦力卡住。")
                print(f"   正在冷静 2 秒后，对该关节重新执行一遍完整的自动调零...")
                time.sleep(2)
                # 第二次重试调零
                result = smart_homing(motor, i, params[0], params[1], params[2])
            # ------------------------------------
            
            # 终极安全校验：如果不是 True，说明彻底失败（寻零超时，或重试后依然失败）
            if result != True:
                print("\n⚠️ 调零或回退最终失败，为防止机械臂干涉，已安全终止后续关节。")
                sys.exit(1)

    # ================= 动态监控：执行最终特定待机姿态 =================
    print("\n[5/5] 正在顺序将所有关节移动至目标全局待机姿态...")
    
    # 包含 1 到 7 轴
    for i in range(1, 8):
        motor = controller.motors.get(i)
        if not motor:
            continue
            
        target = final_target_positions.get(i)
        if target is not None:
            print(f"\n---> 正在控制 Motor [{i}] 移动至最终位置: {target}° ...")
            motor.set_position(target)
            
            # 加入闭环到达判定，拒绝盲目等待
            reach_timeout = 40.0
            start_t = time.time()
            reached = False
            
            while True:
                elapsed = time.time() - start_t
                if elapsed > reach_timeout:
                    break
                    
                curr_pos = motor.position
                diff = abs(curr_pos - target)
                
                sys.stdout.write(f"\r     Motor [{i}] 移动监控 -> 当前: {curr_pos:6.1f}°, 目标: {target:6.1f}° (偏差: {diff:5.1f}°)")
                sys.stdout.flush()
                
                if diff < 1.5:
                    reached = True
                    break
                    
                time.sleep(0.1)
                
            print() # 换行
            if not reached:
                print(f"     ⚠️ 警告: Motor [{i}] 移动超时，未能精准到达最终位置！(停在 {motor.position:.1f}°)")
                sys.exit(1) 
            else:
                print(f"     ✅ Motor [{i}] 已安全就位。")
                time.sleep(0.5) 

    print("\n=====================================================")
    print("✅ 所有调零与最终姿态归位流程完美结束！")
    print("按 [Ctrl + C] 断开电机动力并安全退出。")
    print("=====================================================")
    # =============================================================
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n接收到退出信号，正在安全失能...")
        for motor in controller.motors.values():
            motor.disable()
            time.sleep(0.05)
        controller.stop()
        print("已安全退出。")

if __name__ == "__main__":
    main()