# DummyX 实体机械臂控制工具

用于六轴机械臂与夹爪的网页控制、示教、数据采集、标定和策略部署。节点 1–6 为关节，节点 7 为夹爪。

## 目录结构

```text
webgui/
├── run.py                       # 统一启动入口
├── core/                        # 电机协议、控制器、资源路径
├── apps/                        # 网页、命令行、键盘、采集、部署
├── tools/                       # 回零、相机、标定、固件和总线工具
├── config/motors.yaml            # 电机节点与减速比
├── scripts/bringup_canfd.sh      # CAN-FD 接口配置
├── firmware/                    # 电机固件
├── robot_description/
│   ├── dummy_real_v3/            # ROS 2 描述包、URDF、网格
│   └── legacy_meshes/            # 原根目录网格备份，现有加载器不使用
├── datasets/                    # 原有采集数据与新 episode，忽略上传
├── runtime/                     # 轨迹录制与实验统计，忽略上传
├── tests/                       # 不连接硬件的路径与迁移验证
└── requirements.txt             # Python 依赖
```

## 环境准备

版本采集自本机 Conda 环境 `dummyx_vla`，Python 3.10.20。依赖清单列出项目直接依赖，并非完整环境锁文件；尚未在新建环境中验证安装。

```bash
conda activate dummyx_vla
python -m pip install -r requirements.txt
```

标定使用 ArUco，因此使用 `opencv-contrib-python`，无需同时安装 `opencv-python`。ROS 2 描述包的依赖由系统 ROS 环境提供，见包内 `package.xml`。

## 启动

以下命令在 `webgui/` 中执行。脚本会驱动真实机械臂，运行前确认硬件、限位和急停状态。

```bash
# 配置 can0：仲裁速率 1 Mbps、数据速率 5 Mbps
bash scripts/bringup_canfd.sh

# 查看可用命令，不连接硬件
python run.py --help

# 网页、命令行、键盘控制
python run.py gui
python run.py cli
python run.py keyboard

# 回零、相机检测、采集
python run.py home
python run.py cameras
python run.py collect

# 标定、矩阵验证、策略部署
python run.py calibrate
python run.py verify
python run.py deploy

# 修改节点工具的帮助，不连接硬件
python run.py can-id --help
```

从其他目录启动时，使用 `run.py` 的实际完整路径，例如：

```bash
python /home/jun/dummyx/webgui/run.py gui
```

此处路径只是本机示例；项目搬移后使用新位置。也可在项目根目录执行 `python -m apps.gui`。子目录中的脚本请通过统一入口或模块方式启动，不使用 `python apps/gui.py`。

## 配置和数据路径

`core/paths.py` 根据自身位置确定项目根目录，不依赖终端当前目录。

- 电机配置：`config/motors.yaml`。
- 采集与部署数据：`datasets/`，保留原有 episode；临时部署数据仍为 `datasets/temp_episode/`。
- 示教录制：`runtime/motor_positions.json`，原记录已迁移。
- 实验统计：`runtime/experiment_metrics.json`，原记录已迁移。
- 固件：`firmware/`。升级命令为 `python run.py dfu <节点编号> <固件文件名>`；只传文件名时从固件目录查找，显式相对路径按终端当前目录解析，绝对路径保持原意。

策略服务器、相机序列号、提示词和待机姿态仍在 `apps/deploy_real_arm_vanilla.py` 顶部。标定参数和矩阵分别位于 `tools/calibrate_real.py` 和 `tools/verify_matrix.py`。本次整理保留这些硬件参数。

## 机器人模型

ROS 2 包位于 `robot_description/dummy_real_v3/`，包名仍为 `dummy_real_v3`。在 ROS 工作区构建、加载环境后，可使用：

```bash
ros2 launch dummy_real_v3 display.launch.py
ros2 launch dummy_real_v3 gazebo.launch.py
```

URDF 网格引用为 `package://dummy_real_v3/meshes/...`。MuJoCo 通过 `core.paths.load_mujoco_model()` 在内存中解析为实际文件路径，不修改 URDF。原根目录的重复网格保存在 `legacy_meshes/`，便于核对历史文件。

## 单位与验证

`Motor.set_position(value)` 的参数按关节输出角度（度）处理，发送前按 `角度 / 360 × 减速比` 换算为电机侧转数；当前位置反馈做反向换算。网页位置输入和发送日志已统一标为度。速度、电流等原始协议字段没有在本次整理中更改或重新解释。

```bash
python -m unittest discover -s tests -v
```

测试覆盖启动入口、路径、模型搬移和语法，不进行电机动作或相机采集。实际硬件运行需另行验证。
