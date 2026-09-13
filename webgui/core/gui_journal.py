"""Bounded, asynchronous GUI diagnostic journal; never sends motor commands."""
from collections import deque
import json
import logging
from logging.handlers import RotatingFileHandler
import math
from pathlib import Path
from queue import Queue, Full
import threading
import time
import uuid


def updated_at(received_at, wall=None, monotonic=None):
    if received_at is None or not math.isfinite(received_at):
        return '尚未收到'
    monotonic = time.monotonic() if monotonic is None else monotonic
    return f'{max(0, monotonic - received_at):.2f} 秒前'


def clean_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(v) for v in value]
    return value


class Journal:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / 'gui-events.jsonl'
        self.session = uuid.uuid4().hex
        self.error = ''
        self.closed = False
        self.queue = Queue(maxsize=2000)
        self.handler = RotatingFileHandler(self.path, maxBytes=5_000_000,
                                           backupCount=5, encoding='utf-8')
        self.handler.setFormatter(logging.Formatter('%(message)s'))
        self.handler.handleError = self._write_error
        self.worker = threading.Thread(target=self._run, daemon=True, name='gui-journal')
        self.worker.start()

    def _write_error(self, record):
        self.error = '日志写入失败，请检查磁盘空间和目录权限'

    def append(self, event):
        if self.closed:
            return
        event = clean_json(dict(event, session=self.session))
        try:
            self.queue.put_nowait(json.dumps(event, ensure_ascii=False, allow_nan=False))
        except Full:
            self.error = '日志队列已满，部分记录未保存'

    def _run(self):
        while True:
            line = self.queue.get()
            try:
                if line is None:
                    return
                self.handler.emit(logging.LogRecord('gui', logging.INFO, '', 0, line, (), None))
            except Exception as error:
                self.error = str(error)
            finally:
                self.queue.task_done()

    def recent(self, count=200, session=None):
        """Read only the bounded local rotation set, oldest to newest."""
        result = deque(maxlen=count)
        for path in [Path(str(self.path) + f'.{i}') for i in range(5, 0, -1)] + [self.path]:
            try:
                with path.open(encoding='utf-8') as source:
                    for line in source:
                        try:
                            item = json.loads(line)
                            if isinstance(item, dict) and (session is None or item.get('session') == session):
                                result.append(item)
                        except (ValueError, TypeError):
                            continue
            except FileNotFoundError:
                continue
        return list(result)

    def flush(self, timeout=3.0):
        """Wait briefly for queued records without ever hanging the GUI."""
        deadline = time.monotonic() + timeout
        while self.queue.unfinished_tasks and self.worker.is_alive():
            if time.monotonic() >= deadline:
                self.error = '日志刷盘超时，导出内容可能缺少最后几条记录'
                return False
            time.sleep(.01)
        if self.queue.unfinished_tasks:
            self.error = '日志线程已停止，部分记录未保存'
            return False
        self.handler.flush()
        return True

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.flush()
        try:
            self.queue.put(None, timeout=.5)
        except Full:
            self.error = '日志队列未能正常关闭'
        self.worker.join(timeout=2)
        self.handler.close()
