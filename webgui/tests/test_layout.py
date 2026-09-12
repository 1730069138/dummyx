"""验证目录迁移后的启动和资源路径，不连接硬件。"""
import ast
import os
from pathlib import Path
import shutil
import subprocess
import sys
import struct
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class LayoutTests(unittest.TestCase):
    def test_motor_import_and_position_conversion(self):
        """使用假总线验证移动后的模块导入和位置换算，不发送真实报文。"""
        from unittest.mock import Mock
        from core.motor import Motor
        from core.motorcontroller import MotorController

        self.assertTrue(callable(MotorController))
        bus = Mock()
        motor = Motor(bus, node_id=1, reduction=80)
        motor.set_position(90)
        message = bus.send.call_args.args[0]
        self.assertAlmostEqual(struct.unpack('<f', message.data)[0], 20.0)
        self.assertAlmostEqual(motor.turns_to_degrees(20.0), 90.0)

    def test_python_syntax(self):
        """只解析源代码，避免导入时触发硬件初始化。"""
        for directory in ("core", "apps", "tools", "robot_description"):
            for path in (ROOT / directory).rglob("*.py"):
                with self.subTest(path=path):
                    ast.parse(path.read_text(), filename=str(path))

    def test_launcher_from_other_directory(self):
        """从其他目录查看入口与节点工具帮助。"""
        with tempfile.TemporaryDirectory() as directory:
            for args in (["--help"], ["can-id", "--help"]):
                result = subprocess.run(
                    [sys.executable, str(ROOT / "run.py"), *args],
                    cwd=directory, capture_output=True, text=True, timeout=30,
                )
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_launcher_runs_in_multiprocessing_child(self):
        """NiceGUI 自动重载的子进程会用 __mp_main__ 执行入口。"""
        code = (
            "import runpy, sys; "
            f"sys.argv = [{str(ROOT / 'run.py')!r}, '--help']; "
            f"runpy.run_path({str(ROOT / 'run.py')!r}, run_name='__mp_main__')"
        )
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, "-c", code], cwd=directory,
                capture_output=True, text=True, timeout=30,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("gui", result.stdout)

    def test_gui_launcher_preserves_command_for_reloader(self):
        """GUI 父进程不能丢掉供 NiceGUI 子进程重新分派的命令。"""
        import run as launcher

        argv = [str(ROOT / "run.py"), "gui"]
        with patch.object(sys, "argv", argv.copy()), \
                patch.object(launcher.runpy, "run_module") as run_module:
            launcher.main()
            self.assertEqual(sys.argv, argv)
        run_module.assert_called_once_with("apps.gui", run_name="__main__")

    def test_relocated_model_and_resources(self):
        """复制必要资源到新位置，验证没有依赖原机器的绝对路径。"""
        with tempfile.TemporaryDirectory() as directory:
            relocated = Path(directory) / "搬移后的项目"
            for name in ("core", "config", "robot_description"):
                shutil.copytree(ROOT / name, relocated / name,
                                ignore=shutil.ignore_patterns("__pycache__", "*.log", "legacy_meshes"))
            environment = os.environ.copy()
            environment["PYTHONPATH"] = str(relocated)
            code = (
                "from core.paths import *; "
                "assert MOTORS_CONFIG.is_file(); "
                "assert DATASETS_DIR == ROOT / 'datasets'; "
                "prepare_runtime(); assert RECORDING_FILE.parent.is_dir(); "
                "m = load_mujoco_model(); assert m.nmesh == 11; assert m.nq > 0"
            )
            result = subprocess.run([sys.executable, "-c", code], cwd=directory,
                                    env=environment, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
