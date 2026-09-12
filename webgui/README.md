# DummyX 实体机械臂控制工具

本目录包含 DummyX 六轴机械臂与夹爪的实体控制、标定、数据采集和策略部署工具。

> **安全提示**：本项目会通过 CAN-FD 直接驱动真实机械臂。运行回零、标定、轨迹回放或策略部署前，请确认急停可用、机械臂周围无人、关节限位正确，并先使用低速度测试。

## 硬件与软件环境

- 六个机械臂关节电机：节点 ID 1–6
- 一个夹爪电机：节点 ID 7
- CAN-FD 接口：默认 `can0`
- Intel RealSense D415 相机（相机相关功能需要）
- Ubuntu/Linux
- Python 3.10（当前验证环境：Conda `dummyx_vla`）
- ROS 2 Humble（仅机器人描述、RViz/Gazebo 等功能需要）

电机减速比保存在 `motors.yaml`：节点 1–3 为 80，节点 4–7 为 50。

## 主要文件

| 文件或目录 | 功能 |
| --- | --- |
| `gui.py` | NiceGUI 网页控制台：电机控制、回零、标定、示教、相机和实时曲线 |
| `motor.py` | 单个电机的 CAN 命令、状态解析和单位换算 |
| `motorcontroller.py` | CAN 总线与多电机管理 |
| `auto_homing.py` | 自动寻零和归位 |
| `cli.py` | 交互式命令行控制 |
| `keyboard_control.py` | 键盘控制 |
| `collect_data.py` | 多相机与关节状态同步采集、轨迹回放 |
| `deploy_real_arm_vanilla.py` | 连接远程策略服务并在实体机械臂上执行动作 |
| `find_cameras.py` | 检测 RealSense 相机及序列号 |
| `calibrate_real.py` | 实体机械臂与相机标定 |
| `verify_matrix.py` | 标定矩阵与模型验证 |
| `change_can_id.py` | 扫描和修改电机 CAN 节点 ID |
| `dfu.py` | 电机固件升级 |
| `delay.py` | CAN 通信延迟测试 |
| `dummy_real_v3/` | ROS 2 URDF、网格和 RViz/Gazebo 启动文件 |
| `datasets/` | 采集生成的 episode 数据，不提交到 Git |

## 安装

```bash
conda activate dummyx_vla
python -m pip install -r requirements.txt
```

ROS 2 相关依赖不写入 `requirements.txt`，应通过系统的 ROS 2 环境安装和加载。

## 启动 CAN-FD

脚本会配置 `can0`：仲裁速率 1 Mbps、数据速率 5 Mbps，并启用 CAN-FD。

```bash
chmod +x bringup_canfd.sh
./bringup_canfd.sh
ip -details link show can0
```

## 常用命令

```bash
# 网页控制台
python gui.py

# 交互式命令行
python cli.py

# 自动回零
python auto_homing.py

# 查找 RealSense 相机
python find_cameras.py

# 采集数据
python collect_data.py
```

部署策略前，请先检查 `deploy_real_arm_vanilla.py` 顶部的策略服务器地址、相机序列号、提示词和待机姿态，然后运行：

```bash
python deploy_real_arm_vanilla.py
```

## 位置单位

当前 Python 控制接口使用关节输出角度（degree）：

- `Motor.set_position(value)` 接收关节角度；
- 发送到电机前，代码根据减速比将角度换算为电机侧转数；
- 电机反馈的位置从电机侧转数换算回关节角度。

因此，界面、日志和数据集中的位置字段应统一理解为“度”。现有界面中仍有少量 `turns` 旧标注，后续整理代码时需要修正。

## 上传与数据管理

采集图像、大型 episode 数据、Python 缓存、编辑器配置和运行日志不应提交到 Git。正式上传前需要检查暂存区，确保只包含计划公开的源码、配置示例、机器人模型和必要固件。
