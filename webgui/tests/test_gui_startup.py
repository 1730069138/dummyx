"""Build the real NiceGUI surface with an unavailable CAN bus."""
from pathlib import Path
import subprocess
import sys
import unittest


class GuiStartupTests(unittest.TestCase):
    def test_offline_gui_builds_and_remains_interlocked(self):
        code = '''
import runpy
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch
import core.paths
from nicegui import ui

with tempfile.TemporaryDirectory() as temporary:
    with patch.object(core.paths, 'RUNTIME_DIR', Path(temporary)), \\
         patch('can.Bus', side_effect=RuntimeError('test offline')), \\
         patch.object(ui, 'run') as server, \\
         patch.object(sys, 'argv', ['apps.gui']):
        page = runpy.run_module('apps.gui', run_name='__main__')
        try:
            page['refresh']()
            assert len(page['status_table'].rows) == 7
            assert all(r['status_updated'] == '尚未收到' for r in page['status_table'].rows)
            assert len(page['exit_progress'].rows) == 7
            assert page['alarm_table'].rows
            assert all(r['enabled'] == '未知' for r in page['status_table'].rows)
            assert all(not widget.enabled for widget in page['motion_widgets'])
            assert not page['enable_button'].enabled
            assert not page['homing_button'].enabled
            assert all(not widget.enabled for widget in page['commissioning_widgets'])
            assert 'CAN' in page['control'].reason()
            assert server.call_args.kwargs['reload'] is False
            assert server.call_args.kwargs['show'] is False
            page['control'].active = 'test-task'
            page['disconnected']()
            assert page['control'].latched
            assert page['control'].cancel_event.is_set()
            page['refresh']()
            assert page['exit_panel'].visible
            page['cancel_exit']()
            page['refresh']()
            assert not page['exit_panel'].visible
            import signal
            page['receive_exit_signal'](signal.SIGTERM, None)
            page['refresh']()
            assert page['exit_panel'].visible
            import asyncio
            from nicegui import app
            with patch.object(app, 'shutdown') as stop, patch.object(ui, 'notify'):
                asyncio.run(page['finish_exit'](True))
                stop.assert_not_called()  # offline return must not exit
                page['control'].active = None
                asyncio.run(page['finish_exit'](False))
                stop.assert_called_once()
            with patch.object(ui, 'download') as download:
                asyncio.run(page['export_diagnostics']())
                import json
                exported = json.loads(download.call_args.args[0])
                assert exported['events']
                assert all(e['session'] == exported['session'] for e in exported['events'])
        finally:
            page['shutdown']()
'''
        result = subprocess.run([sys.executable, '-c', code],
                                cwd=Path(__file__).resolve().parents[1],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
