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
    module = COMMANDS[options.command]
    # NiceGUI's auto-reloader starts a fresh process which executes this file
    # as ``__mp_main__`` and reuses the current argv.  Keep the ``gui`` command
    # intact so that child process can dispatch to apps.gui again.
    if options.command != "gui":
        sys.argv = [module, *options.args]
    runpy.run_module(module, run_name="__main__")


if __name__ in {"__main__", "__mp_main__"}:
    main()
