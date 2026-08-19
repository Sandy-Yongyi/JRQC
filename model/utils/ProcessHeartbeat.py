import threading
import time
from contextlib import contextmanager


class ProcessHeartbeat:
    """按固定最小间隔更新子进程共享心跳。"""

    def __init__(self, shared_timestamp=None, interval_s: float = 1.0):
        self.shared_timestamp = shared_timestamp
        self.interval_s = max(float(interval_s), 0.001)
        self._last_touch = 0.0

    def touch(self, force: bool = False) -> None:
        now = time.monotonic()
        if self.shared_timestamp is not None and (force or now - self._last_touch >= self.interval_s):
            self.shared_timestamp.value = now
            self._last_touch = now

    def sleep(self, seconds: float) -> None:
        """长时间等待时继续刷新心跳，避免把正常重连误判为进程卡死。"""
        deadline = time.monotonic() + max(float(seconds), 0.0)
        while True:
            self.touch()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(self.interval_s, remaining))

    @contextmanager
    def keep_alive(self, max_duration_s: float = 30.0):
        """已知长操作在有限租约内刷新心跳，避免永久掩盖真正的死锁。"""
        if self.shared_timestamp is None:
            yield
            return

        stop_event = threading.Event()
        lease_s = max(float(max_duration_s), self.interval_s)

        def pulse():
            deadline = time.monotonic() + lease_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or stop_event.wait(min(self.interval_s, remaining)):
                    return
                self.touch(force=True)

        self.touch(force=True)
        thread = threading.Thread(target=pulse, daemon=True)
        thread.start()
        try:
            yield
        finally:
            stop_event.set()
            thread.join(timeout=self.interval_s)
            self.touch(force=True)
