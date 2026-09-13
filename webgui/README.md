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

## GUI 第一阶段控制改造

默认打开 `http://127.0.0.1:8080`，关闭自动重载与自动打开浏览器。
仅验证界面、不连接实体机械臂时运行 `python run.py gui --virtual`。
虚拟 CAN 不模拟驱动器反馈，因此默认保持离线、禁止运动。可用 `--port 18081` 指定端口。
需要局域网访问时显式指定 `--host 0.0.0.0`；当前还没有登录与控制权分配，
只能在受控网络使用。任一浏览器断连会取消当前任务；断连检测依赖网络超时，不是实时安全功能。

`config/motors.yaml` 新增每关节 `limits.min_deg`、`limits.max_deg`、`limits.verified`。
填写经过实机核实、与当前零位一致的输出轴角度范围后，将 `verified` 设为 `true` 并重启。
未配置范围时仍允许执行专用自动碰撞调零，但禁止普通使能和运动；不会猜测限位。
软限位只检查当前关节角度与目标角度，不检查整条路径的自碰撞、环境碰撞或安全速度。

单关节定位、往返循环和逐点回放经过统一任务入口，禁止并发运动任务。
轨迹开始前预检全部点；执行中分别检查状态和位置反馈是否超过 2 秒。
每点必须收到命令之后的新状态与位置反馈，目标到位标志有效且角度误差不超过 1°，
才继续下一点；30 秒未到位则失败并锁定后续指令。
这些时间与容差是软件监控参数，尚未经过实机工况验证。

- **取消当前任务**：中断等待、禁止发送后续指令并锁定新任务，不能撤回已下发的目标。
- **全部失能**：先取消任务，再逐关节发送失能；一关节发送失败仍尝试其他关节。
  发送成功不表示机械臂已停止，必须检查新鲜驱动器反馈；无抱闸时可能因重力下坠。
- **解除软件锁定**：检查通信和故障后恢复接受新任务，不自动恢复旧任务或使能电机。
- **复位驱动器故障**：要求所有关节已失能，发送复位后继续保持软件锁定。

自动碰撞调零不依赖调零前关节范围。它沿用原机械臂各关节的寻零方向、低电流、
回退位置、J5 预调零和 J7 降速设置，并加入单任务锁、反馈新鲜度监测、可取消和失败锁定。
起步已压在限位上时会确认故障清除和重新使能，再向搜索反方向回退 30°。
未调零坐标不作为绝对坐标使用；搜索目标由命令下发时的当前位置加上完整搜索行程计算。
已配置软限位时，搜索行程至少覆盖软限位跨度并额外增加 10°，例如 J6 使用 290° 而非固定目标 -180°。
停转候选必须由多帧独立的速度、电流、位置和状态反馈持续确认；随后反向退出 8°，
以配置的验证电流再次接近。两次停转位置误差超过 2° 时拒绝写入零点，避免把静摩擦、
重力负载或电流不足造成的中途停转误判为机械限位。相关阈值位于 `homing_defaults`。
取消或失败时会尝试失能当前调零关节，但软件动作不能替代硬件急停。
软限位是调零后坐标系中的安全工作范围，只有全部关节完成本次启动的调零后才可用于普通控制。
全部调零后可在“配置与维护”中使用软限位调试：一次只允许一个关节移动最多 2°，
临时把速度与加速度降到配置值，并按寻零方向限制在带 2° 余量的搜索行程内。
可采集当前位置作为最小值和最大值；二次确认后以原子替换方式写入 `config/motors.yaml`，
标记为已核实并立即参与普通运动联锁。临时包络不检测自碰撞或实际另一端机械止挡。
设置零点以外的底层标定、手动阻尼切换及通用保护参数写入仍暂时禁用。
原 Ready / Max. Range / 夹爪预设角度也未作为已验证动作恢复。
需要确认实际回零方式、速度与电流单位、停止及抱闸策略后再接入受控任务。

示教记录为 2 Hz 位置采样，最多 10000 点；失联自动结束采样，已有点仍可保存。
回放是等待到位后再执行下一点的逐点运动，不复现原始速度或时间轨迹。
相机默认 30 FPS；位置曲线单位为度，过期反馈画成断点。
“归位、确认失能后退出”会先确认各关节进入结束姿态（实际位置允许目标 ±1°），
再重复发送失能并取得全部关节的新鲜失能反馈，最后关闭 GUI、CAN 并结束终端命令。
归位或失能确认失败时保留程序运行并锁定后续运动；“不归位，直接退出”不会主动失能。
硬件急停未接入 GUI；此版本属于软件操作改进，未经过工业安全认证。
