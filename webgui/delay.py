import can
import time
import struct
from datetime import datetime

def test_can_latency():
    # CAN总线配置
    node_id = 2  # 默认节点ID
    can_interface = 'socketcan'  # Linux系统使用socketcan
    # can_interface = 'can0'     # 或者直接指定接口名
    # Windows系统可能需要使用其他接口类型，如:
    # can_interface = 'pcan'    # PCAN接口
    # can_interface = 'ixxat'    # IXXAT接口
    
    # 初始化CAN总线
    try:
        # bus = can.interface.Bus(channel=can_interface, bustype='socketcan', bitrate=500000)
        bus = can.Bus(interface='socketcan', channel='can0')
        print("CAN总线初始化成功")
    except Exception as e:
        print(f"CAN总线初始化失败: {e}")
        return

    # 定义CAN命令ID
    CAN_CMD_MOTOR_ENABLE = 1
    
    # 创建消息ID (根据手册第15页的格式)
    # dir(1bit) | node_id(5bit) | cmd_id(5bit)
    msg_id = (0 << 10) | ((node_id & 0x1F) << 5) | (CAN_CMD_MOTOR_ENABLE & 0x1F)
    
    # 创建CAN消息 (无数据内容)
    message = can.Message(
        arbitration_id=msg_id,
        data=[],
        is_extended_id=False
    )
    
    # 设置接收过滤器，只接收来自我们节点的响应
    response_id = (1 << 10) | ((node_id & 0x1F) << 5) | (CAN_CMD_MOTOR_ENABLE & 0x1F)
    bus.set_filters([{"can_id": response_id, "can_mask": 0x7FF, "extended": False}])
    
    # 测试次数
    test_count = 10
    total_latency = 0
    min_latency = float('inf')
    max_latency = 0
    
    print(f"开始测试，将进行{test_count}次CAN_CMD_MOTOR_ENABLE命令的往返时延测量...")
    test_results = []    
    
    for i in range(test_count):
        try:
            # 发送前的时间戳
            send_time = time.perf_counter()
            
            # 发送消息
            bus.send(message)
            # print(f"测试 {i+1}/{test_count}: 已发送CAN_CMD_MOTOR_ENABLE命令")
            
            # 等待响应
            response = bus.recv(timeout=1.0)  # 1秒超时
            
            if response is None:
                print("  错误: 未收到响应")
                continue
                
            # 接收后的时间戳
            recv_time = time.perf_counter()
            
            # 计算时延(毫秒)
            latency = (recv_time - send_time) * 1000
            
            # 更新统计数据
            total_latency += latency
            min_latency = min(min_latency, latency)
            max_latency = max(max_latency, latency)
            
            print(f"  收到响应，往返时延: {latency:.3f} ms")
            
            # 解析响应数据
            if len(response.data) >= 4:
                result = struct.unpack('<i', response.data[0:4])[0]
                # print(f"  执行结果: {result} (0表示成功)")
            
            # 短暂暂停
            time.sleep(0.1)
            
        except can.CanError as e:
            print(f"CAN通信错误: {e}")
            break
    
    # 关闭CAN总线
    bus.shutdown()
    
    # 输出统计结果
    if test_count > 0:
        avg_latency = total_latency / test_count
        print("\n测试结果:")
        print(f"  平均往返时延: {avg_latency:.3f} ms, 往返频率: {1000 / avg_latency:.3f}Hz, 单向频率: {(1000 / avg_latency)*2:.3f}Hz")
        print(f"  最小往返时延: {min_latency:.3f} ms, 往返最小频率: {1000 / min_latency:.3f}Hz, 单向最小频率: {1000 / min_latency *2:.3f}Hz")
        print(f"  最大往返时延: {max_latency:.3f} ms, 往返最大频率: {1000 / max_latency:.3f}Hz, 单向最大频率: {1000 / max_latency *2:.3f}Hz")
    else:
        print("未完成有效测试")

if __name__ == "__main__":
    test_can_latency()
