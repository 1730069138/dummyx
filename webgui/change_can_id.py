#!/usr/bin/env python3
"""
Rdrive Motor Controller - CAN ID 修改工具

CAN 协议说明:
  - 11-bit 标准 CAN ID 格式: [1-bit echo][5-bit node_id][5-bit cmd]
  - echo bit (bit10): 0=请求, 1=响应
  - node_id (bit9-5): 设备节点 ID (1~31), 0x1F(31)=广播
  - cmd (bit4-0): 命令码

  SET_CONFIG 命令 (cmd=17): dlc=8, data[0:4]=配置索引(1-based), data[4:8]=新值
  GET_CONFIG 命令 (cmd=18): dlc=4, data[0:4]=配置索引(1-based)
  SAVE_ALL_CONFIG 命令 (cmd=19): dlc=0, 保存所有配置到 Flash

  node_id 在 tUsrConfig 结构中的 1-based 索引 = 30

依赖: python-can
  pip install python-can
"""

import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    print("错误: 请先安装 python-can 库")
    print("  pip install python-can")
    sys.exit(1)


# ============================================================================
# CAN 协议常量 (来自 can.h)
# ============================================================================

# CAN ID 位域定义
ID_ECHO_BIT = 0x400  # bit10
ID_NODE_BIT = 0x3E0  # bit9~5
ID_CMD_BIT  = 0x01F  # bit4~0

BROADCAST_NODE_ID = 0x1F  # 广播地址 (31)

# CAN 命令定义 (来自 eCanCmd 枚举)
CAN_CMD_MOTOR_DISABLE   = 0
CAN_CMD_MOTOR_ENABLE    = 1
CAN_CMD_SET_TORQUE      = 2
CAN_CMD_SET_VELOCITY    = 3
CAN_CMD_SET_POSITION    = 4
CAN_CMD_CALIB_START     = 5
CAN_CMD_CALIB_REPORT    = 6
CAN_CMD_CALIB_ABORT     = 7
CAN_CMD_ANTICOGGING_START  = 8
CAN_CMD_ANTICOGGING_REPORT = 9
CAN_CMD_ANTICOGGING_ABORT  = 10
CAN_CMD_SET_HOME        = 11
CAN_CMD_ERROR_RESET     = 12
CAN_CMD_GET_STATUSWORD  = 13
CAN_CMD_STATUSWORD_REPORT = 14
CAN_CMD_GET_VALUE_1     = 15
CAN_CMD_GET_VALUE_2     = 16
CAN_CMD_SET_CONFIG      = 17
CAN_CMD_GET_CONFIG      = 18
CAN_CMD_SAVE_ALL_CONFIG = 19
CAN_CMD_RESET_ALL_CONFIG = 20
CAN_CMD_SYNC            = 21
CAN_CMD_HEARTBEAT       = 22
CAN_CMD_GET_FW_VERSION  = 28

# tUsrConfig 配置索引 (1-based, 用于 SET_CONFIG / GET_CONFIG)
# 按 usr_config.h 中 tUsrConfig 结构体字段顺序排列
CONFIG_INDEX = {
    'invert_motor_dir':       1,
    'inertia':                2,
    'torque_constant':        3,
    'motor_pole_pairs':       4,
    'motor_phase_resistance': 5,
    'motor_phase_inductance': 6,
    'current_limit':          7,
    'velocity_limit':         8,
    'calib_current':          9,
    'calib_voltage':          10,
    'control_mode':           11,
    'pos_gain':               12,
    'vel_gain':               13,
    'vel_integrator_gain':    14,
    'current_ctrl_bw':        15,
    'anticogging_enable':     16,
    'sync_target_enable':     17,
    'target_velcity_window':  18,
    'target_position_window': 19,
    'torque_ramp_rate':       20,
    'velocity_ramp_rate':     21,
    'position_filter_bw':     22,
    'profile_velocity':       23,
    'profile_accel':          24,
    'profile_decel':          25,
    'protect_under_voltage':  26,
    'protect_over_voltage':   27,
    'protect_over_current':   28,
    'protect_i_bus_max':      29,
    'node_id':                30,
    'can_baudrate':           31,
    'heartbeat_consumer_ms':  32,
    'heartbeat_producer_ms':  33,
}

CONFIG_NODE_ID = CONFIG_INDEX['node_id']  # = 30


# ============================================================================
# 辅助函数 (对应固件中 util.c 的数据转换)
# ============================================================================

def int32_to_bytes(val: int) -> bytes:
    """int32 转 4 字节 (小端序)"""
    return struct.pack('<i', val)


def uint32_to_bytes(val: int) -> bytes:
    """uint32 转 4 字节 (小端序)"""
    return struct.pack('<I', val)


def bytes_to_int32(data: bytes) -> int:
    """4 字节 (小端序) 转 int32"""
    return struct.unpack('<i', data[:4])[0]


def bytes_to_uint32(data: bytes) -> int:
    """4 字节 (小端序) 转 uint32"""
    return struct.unpack('<I', data[:4])[0]


def make_can_id(node_id: int, cmd: int, echo: bool = False) -> int:
    """
    构造 11-bit 标准 CAN ID
    格式: [1-bit echo][5-bit node_id][5-bit cmd]
    """
    can_id = (node_id & 0x1F) << 5 | (cmd & 0x1F)
    if echo:
        can_id |= ID_ECHO_BIT
    return can_id


def parse_can_id(can_id: int) -> dict:
    """解析 CAN ID"""
    return {
        'echo':    bool(can_id & ID_ECHO_BIT),
        'node_id': (can_id & ID_NODE_BIT) >> 5,
        'cmd':     can_id & ID_CMD_BIT,
    }


# ============================================================================
# Rdrive CAN 通信类
# ============================================================================

class RdriveController:
    """Rdrive 电机控制器 CAN 通信"""

    def __init__(self, interface='socketcan', channel='can0'):
        """
        初始化 CAN 总线连接 (参考 controller.py)

        Args:
            interface: CAN 接口类型 ('socketcan', 'slcan', 'pcan', 'kvaser', etc.)
            channel:   CAN 通道 (Linux: 'can0', Windows PCAN: 'PCAN_USBBUS1',
                       Serial: '/dev/ttyACM0' 或 'COM3')
        """
        self.timeout = 1.0  # 响应超时时间 (秒)

        try:
            self.bus = can.Bus(
                interface=interface,
                channel=channel,
                receive_own_messages=True,
                fd=True,
            )
            print(f"✓ CAN 总线已连接: interface={interface}, channel={channel}")
        except Exception as e:
            print(f"✗ CAN 总线连接失败: {e}")
            raise

    def close(self):
        """关闭 CAN 总线"""
        if hasattr(self, 'bus') and self.bus:
            self.bus.shutdown()
            print("✓ CAN 总线已关闭")

    def send_and_receive(self, node_id: int, cmd: int, data: bytes = b'',
                         timeout: float = None) -> can.Message:
        """
        发送 CAN 帧 并等待响应 (echo)

        Args:
            node_id: 目标节点 ID
            cmd:     命令码
            data:    数据负载 (最多 8 字节)
            timeout: 响应超时时间

        Returns:
            响应 CAN 消息, 超时返回 None
        """
        if timeout is None:
            timeout = self.timeout

        tx_id = make_can_id(node_id, cmd, echo=False)
        expected_rx_id = make_can_id(node_id, cmd, echo=True)

        msg = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False,
        )

        # 清空接收缓冲区
        while self.bus.recv(timeout=0.01):
            pass

        self.bus.send(msg)

        # 等待匹配的响应帧
        start = time.time()
        while time.time() - start < timeout:
            rx_msg = self.bus.recv(timeout=timeout - (time.time() - start))
            if rx_msg is None:
                break
            # 检查是否为匹配的 echo 帧
            rx_parsed = parse_can_id(rx_msg.arbitration_id)
            if (rx_parsed['echo'] and
                rx_parsed['cmd'] == cmd and
                (rx_parsed['node_id'] == node_id or node_id == BROADCAST_NODE_ID)):
                return rx_msg

        return None

    def get_config(self, node_id: int, config_index: int) -> int:
        """
        读取配置项

        Args:
            node_id:      目标节点 ID
            config_index: 配置索引 (1-based)

        Returns:
            配置值 (int32), 失败返回 None
        """
        data = int32_to_bytes(config_index)
        resp = self.send_and_receive(node_id, CAN_CMD_GET_CONFIG, data)

        if resp and len(resp.data) >= 8:
            idx = bytes_to_int32(bytes(resp.data[0:4]))
            val = bytes_to_int32(bytes(resp.data[4:8]))
            if idx == config_index:
                return val
            elif idx == -1:
                print(f"  ✗ 配置索引 {config_index} 无效")
                return None

        return None

    def set_config(self, node_id: int, config_index: int, value: int) -> bool:
        """
        设置配置项 (仅修改 RAM, 未保存到 Flash)

        Args:
            node_id:      目标节点 ID
            config_index: 配置索引 (1-based)
            value:        新值 (int32)

        Returns:
            设置成功返回 True
        """
        data = int32_to_bytes(config_index) + int32_to_bytes(value)
        resp = self.send_and_receive(node_id, CAN_CMD_SET_CONFIG, data)

        if resp and len(resp.data) >= 8:
            idx = bytes_to_int32(bytes(resp.data[0:4]))
            if idx == config_index:
                return True
            elif idx == -1:
                print(f"  ✗ 配置索引 {config_index} 无效")
                return False

        return False

    def save_all_config(self, node_id: int) -> bool:
        """
        保存所有配置到 Flash

        Args:
            node_id: 目标节点 ID

        Returns:
            保存成功返回 True
        """
        # 修改点 3：延长超时时间至 5.0 秒，兼容 Flash 擦除较慢的固件
        resp = self.send_and_receive(node_id, CAN_CMD_SAVE_ALL_CONFIG, b'', timeout=5.0)

        if resp is not None:
            if len(resp.data) >= 4:
                ret = bytes_to_int32(bytes(resp.data[0:4]))
                return ret == 0
            else:
                # 兼容固件只回复 dlc=0 的 echo 帧作为确认的情况
                return True

        return False

    def get_fw_version(self, node_id: int) -> tuple:
        """
        获取固件版本号

        Returns:
            (major, minor) 或 None
        """
        resp = self.send_and_receive(node_id, CAN_CMD_GET_FW_VERSION, b'')

        if resp and len(resp.data) >= 8:
            major = bytes_to_int32(bytes(resp.data[0:4]))
            minor = bytes_to_int32(bytes(resp.data[4:8]))
            return (major, minor)

        return None

    def get_node_id(self, node_id: int) -> int:
        """读取当前 node_id 配置"""
        return self.get_config(node_id, CONFIG_NODE_ID)

    def change_node_id(self, current_id: int, new_id: int, save: bool = True) -> bool:
        """
        修改 CAN 节点 ID

        步骤:
          1. 读取当前 node_id (验证通信)
          2. 设置新 node_id (写入 RAM)
          3. (可选) 保存到 Flash

        Args:
            current_id: 当前节点 ID (1~31)
            new_id:     新节点 ID (1~31)
            save:       是否保存到 Flash (默认 True)

        Returns:
            修改成功返回 True
        """
        if not (1 <= new_id <= 31):
            print(f"✗ 新 ID {new_id} 超出范围 (1~31)")
            return False

        if not (1 <= current_id <= 31):
            print(f"✗ 当前 ID {current_id} 超出范围 (1~31)")
            return False

        if current_id == new_id:
            print(f"✗ 新 ID 与当前 ID 相同 ({current_id})")
            return False

        # Step 1: 验证通信, 读取当前 node_id
        print(f"\n[1/3] 读取当前节点 ID (node_id={current_id}) ...")
        read_id = self.get_node_id(current_id)
        if read_id is None:
            print(f"  ✗ 无法与节点 {current_id} 通信, 请检查:")
            print(f"    - 设备是否上电")
            print(f"    - CAN 总线连接是否正常")
            print(f"    - 波特率是否匹配")
            print(f"    - 节点 ID 是否正确")
            return False
        print(f"  ✓ 当前节点 ID = {read_id}")

        if read_id != current_id:
            print(f"  ⚠ 读取到的 ID ({read_id}) 与预期 ({current_id}) 不匹配")

        # Step 2: 设置新 node_id
        print(f"\n[2/3] 设置新节点 ID: {current_id} → {new_id} ...")
        success = self.set_config(current_id, CONFIG_NODE_ID, new_id)
        if not success:
            print(f"  ✗ 设置新 node_id 失败")
            return False
        print(f"  ✓ 新 node_id 已写入 RAM")

        print(f"  ⏳ 等待控制器响应 ...")
        time.sleep(0.5)

        # Step 3: 保存到 Flash
        if save:
            print(f"\n[3/3] 保存配置到 Flash ...")
            # 修改点 1：先尝试用新 ID 发送保存指令
            success = self.save_all_config(new_id)
            if not success:
                # 如果超时，说明固件尚未将新 ID 应用到 CAN 硬件滤波器，回退用旧 ID 发送
                print(f"  ⚠ 使用新 ID={new_id} 通信超时，尝试使用当前 ID={current_id} 发送保存指令 ...")
                success = self.save_all_config(current_id)
                
            if not success:
                print(f"  ✗ 保存配置失败")
                print(f"  ⚠ node_id 已在 RAM 中修改, 但未保存到 Flash")
                print(f"  ⚠ 重新上电后会恢复为旧 ID")
                return False
            print(f"  ✓ 配置已保存到 Flash")
        else:
            print(f"\n[3/3] 跳过保存 (仅修改 RAM, 重启后恢复)")

        # 修改点 2：验证逻辑增强
        print(f"\n验证: 读取配置核对最新 ID ...")
        verify_id = self.get_node_id(new_id)
        if verify_id == new_id:
            print(f"  ✓ 验证通过! 节点 ID 已成功修改为 {new_id}，并且已立即生效。")
            return True
        else:
            # 新 ID 读不到，尝试用旧 ID 读。若旧 ID 读出的内部 node_id 是新 ID，说明修改成功但需要重启。
            verify_id_old = self.get_node_id(current_id)
            if verify_id_old == new_id:
                print(f"  ✓ 验证通过! 内部节点 ID 已经成功保存为 {new_id}。")
                print(f"  🔔 提示: 该固件版本需要【断电重启 (Power Cycle)】控制器后，新 ID ({new_id}) 才会真正在 CAN 总线上生效！")
                return True
            else:
                print(f"  ✗ 验证失败, 读取不到正确的 ID。")
                return False


# ============================================================================
# 扫描总线上的设备
# ============================================================================

def scan_bus(controller: RdriveController, id_range: range = range(1, 32)):
    """扫描 CAN 总线上的 Rdrive 设备"""
    print(f"\n扫描 CAN 总线上的 Rdrive 设备 (ID: {id_range.start}~{id_range.stop - 1}) ...")
    found = []

    for node_id in id_range:
        fw_ver = controller.get_fw_version(node_id)
        if fw_ver:
            current_id = controller.get_node_id(node_id)
            found.append({
                'node_id': node_id,
                'config_id': current_id,
                'fw_version': fw_ver,
            })
            print(f"  ✓ 发现设备: ID={node_id}, 固件版本=v{fw_ver[0]}.{fw_ver[1]}")

    if not found:
        print("  未发现任何设备")

    return found


# ============================================================================
# 主程序
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Rdrive 电机控制器 - CAN ID 修改工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 扫描总线上的设备
  %(prog)s --scan

  # 将 ID=1 的设备修改为 ID=3
  %(prog)s --current-id 1 --new-id 3

  # 仅修改 RAM (不保存到 Flash, 重启后恢复)
  %(prog)s --current-id 1 --new-id 3 --no-save

  # 指定 CAN 接口 (Linux SocketCAN)
  %(prog)s --interface socketcan --channel can0 --current-id 1 --new-id 3

  # 指定 CAN 接口 (USB-CAN 串口设备, slcan)
  %(prog)s --interface slcan --channel /dev/ttyACM0 --current-id 1 --new-id 3

  # 指定 CAN 接口 (Windows PCAN)
  %(prog)s --interface pcan --channel PCAN_USBBUS1 --current-id 1 --new-id 3

  # 读取指定设备的当前 ID
  %(prog)s --read-id 1

CAN 接口配置 (常见):
  Linux SocketCAN:   --interface socketcan --channel can0
  USB-CAN (slcan):   --interface slcan --channel /dev/ttyACM0
  PCAN (Windows):    --interface pcan --channel PCAN_USBBUS1
  CANable:           --interface slcan --channel /dev/ttyACM0
  Kvaser:            --interface kvaser --channel 0
        """
    )

    # CAN 接口参数
    parser.add_argument('--interface', '-i', default='socketcan',
                        help='CAN 接口类型 (默认: socketcan)')
    parser.add_argument('--channel', '-c', default='can0',
                        help='CAN 通道 (默认: can0)')

    # 操作参数
    parser.add_argument('--scan', action='store_true',
                        help='扫描总线上的所有设备')
    parser.add_argument('--read-id', type=int, metavar='NODE_ID',
                        help='读取指定设备的当前 node_id')
    parser.add_argument('--current-id', type=int, metavar='ID',
                        help='当前设备的节点 ID (1~31)')
    parser.add_argument('--new-id', type=int, metavar='ID',
                        help='新的节点 ID (1~31)')
    parser.add_argument('--no-save', action='store_true',
                        help='不保存到 Flash (仅修改 RAM)')

    args = parser.parse_args()

    # 参数校验
    if not args.scan and args.read_id is None and (args.current_id is None or args.new_id is None):
        parser.print_help()
        print("\n错误: 请指定操作 (--scan / --read-id / --current-id + --new-id)")
        sys.exit(1)

    # 连接 CAN 总线
    try:
        ctrl = RdriveController(
            interface=args.interface,
            channel=args.channel,
        )
    except Exception:
        sys.exit(1)

    try:
        # 扫描
        if args.scan:
            devices = scan_bus(ctrl)
            if devices:
                print(f"\n共发现 {len(devices)} 个设备:")
                print(f"  {'ID':>4s}  {'固件版本':>10s}")
                print(f"  {'----':>4s}  {'----------':>10s}")
                for dev in devices:
                    ver = dev['fw_version']
                    print(f"  {dev['node_id']:>4d}  v{ver[0]}.{ver[1]:>8d}")

        # 读取 ID
        elif args.read_id is not None:
            print(f"\n读取设备 (ID={args.read_id}) 的配置 ...")
            fw = ctrl.get_fw_version(args.read_id)
            if fw:
                print(f"  固件版本: v{fw[0]}.{fw[1]}")
            node_id = ctrl.get_node_id(args.read_id)
            if node_id is not None:
                print(f"  当前 node_id = {node_id}")
            else:
                print(f"  ✗ 无法读取, 设备可能不存在或通信失败")

        # 修改 ID
        elif args.current_id is not None and args.new_id is not None:
            print(f"\n{'='*50}")
            print(f" Rdrive CAN ID 修改")
            print(f" 当前 ID: {args.current_id}  →  新 ID: {args.new_id}")
            print(f" 保存到 Flash: {'否 (仅 RAM)' if args.no_save else '是'}")
            print(f"{'='*50}")

            success = ctrl.change_node_id(
                current_id=args.current_id,
                new_id=args.new_id,
                save=not args.no_save,
            )

            if success:
                print(f"\n✅ CAN ID 修改操作完成。")
            else:
                print(f"\n❌ CAN ID 修改失败")
                sys.exit(1)

    finally:
        ctrl.close()


if __name__ == '__main__':
    main()