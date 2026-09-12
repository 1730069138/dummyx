"""从任意工作目录启动项目应用。"""
import argparse
import runpy
import sys

COMMANDS = {
    "gui": "apps.gui",
    "cli": "apps.cli",
    "keyboard": "apps.keyboard_control",
    "collect": "apps.collect_data",
    "deploy": "apps.deploy_real_arm_vanilla",
    "home": "tools.auto_homing",
    "cameras": "tools.find_cameras",
    "calibrate": "tools.calibrate_real",
    "verify": "tools.verify_matrix",
    "can-id": "tools.change_can_id",
    "latency": "tools.delay",
    "dfu": "tools.dfu",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=COMMANDS)
    parser.add_argument("args", nargs=argparse.REMAINDER)
    options = parser.parse_args()
    sys.argv = [COMMANDS[options.command], *options.args]
    runpy.run_module(COMMANDS[options.command], run_name="__main__")


if __name__ == "__main__":
    main()
