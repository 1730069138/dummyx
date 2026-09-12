"""独立于终端当前工作目录的项目资源路径。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MOTORS_CONFIG = ROOT / "config" / "motors.yaml"
DATASETS_DIR = ROOT / "datasets"
RUNTIME_DIR = ROOT / "runtime"
RECORDING_FILE = RUNTIME_DIR / "motor_positions.json"
METRICS_FILE = RUNTIME_DIR / "experiment_metrics.json"
TEMP_EPISODE = DATASETS_DIR / "temp_episode"
ROBOT_DIR = ROOT / "robot_description" / "dummy_real_v3"
URDF_PATH = ROBOT_DIR / "urdf" / "dummy_real_v3.urdf"
FIRMWARE_DIR = ROOT / "firmware"


def prepare_runtime():
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)


def load_mujoco_model():
    """解析 ROS 网格资源地址，供 MuJoCo 加载，保留原始 URDF。"""
    import xml.etree.ElementTree as ET
    import mujoco

    robot = ET.parse(URDF_PATH).getroot()
    prefix = "package://dummy_real_v3/"
    for mesh in robot.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(prefix):
            path = ROBOT_DIR / filename.removeprefix(prefix)
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set("filename", str(path))
    extension = robot.find("mujoco")
    if extension is None:
        extension = ET.SubElement(robot, "mujoco")
    compiler = extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(extension, "compiler")
    compiler.set("strippath", "false")
    return mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
