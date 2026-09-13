import json
from pathlib import Path
import tempfile
import unittest
from core.gui_journal import Journal, updated_at


class JournalTests(unittest.TestCase):
    def test_restart_preserves_history_and_separates_sessions(self):
        with tempfile.TemporaryDirectory() as directory:
            first = Journal(directory)
            first.append({'time': 'first', 'message': 'J2 超时',
                          'details': [{'position_age': float('inf'), 'target': 80}]})
            first.close()
            second = Journal(directory)
            try:
                second.append({'time': 'second', 'message': '重试'})
                second.flush()
                records = second.recent()
                self.assertEqual([r['time'] for r in records], ['first', 'second'])
                self.assertIsNone(records[0]['details'][0]['position_age'])
                self.assertEqual(records[0]['details'][0]['target'], 80)
                self.assertEqual(len(second.recent(session=second.session)), 1)
                for line in second.path.read_text().splitlines():
                    json.loads(line, parse_constant=lambda s: self.fail(s))
            finally:
                second.close()

    def test_rotation_and_incomplete_record_do_not_hide_valid_history(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = Journal(directory)
            journal.handler.maxBytes = 150
            for i in range(3):
                journal.append({'time': str(i), 'message': 'x' * 50})
            journal.close()
            self.assertTrue(Path(str(journal.path) + '.1').exists())
            with journal.path.open('a') as source:
                source.write('{broken')
            self.assertEqual([r['time'] for r in journal.recent()], ['0', '1', '2'])

    def test_unknown_update_is_not_displayed_as_now(self):
        self.assertEqual(updated_at(None), '尚未收到')
        self.assertEqual(updated_at(float('inf')), '尚未收到')
        self.assertEqual(updated_at(90, wall=1000, monotonic=100),
                         updated_at(90, wall=1010, monotonic=110))
