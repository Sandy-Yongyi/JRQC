"""PLC慢周期诊断；只计时，不改变通信或队列行为。"""

import time
import gc
import threading
from contextlib import contextmanager


class PlcCycleTiming:
    def __init__(self):
        self.stages = {}
        self.packets = 0

    @contextmanager
    def stage(self, name, *, track_gc=False):
        wall = time.perf_counter()
        cpu = time.thread_time()
        callback = None
        if track_gc:
            owner = threading.get_ident()
            starts = {}

            def on_gc(phase, info):
                if threading.get_ident() != owner:
                    return
                generation = info["generation"]
                if phase == "start":
                    starts[generation] = (time.perf_counter(), time.thread_time())
                elif generation in starts:
                    started_wall, started_cpu = starts.pop(generation)
                    values = self.stages.setdefault(
                        f"{name}_gc_gen{generation}", [0.0, 0.0, 0]
                    )
                    values[0] += (time.perf_counter() - started_wall) * 1000
                    values[1] += (time.thread_time() - started_cpu) * 1000
                    values[2] += 1

            callback = on_gc
            gc.callbacks.append(callback)
        try:
            yield
        finally:
            if callback is not None:
                gc.callbacks.remove(callback)
            elapsed = (time.perf_counter() - wall) * 1000
            cpu_elapsed = (time.thread_time() - cpu) * 1000
            values = self.stages.setdefault(name, [0.0, 0.0, 0])
            values[0] += elapsed
            values[1] += cpu_elapsed
            values[2] += 1

    def report(self, logger, processing_ms, fifo):
        scan_ms = self.stages.get("scan", [0])[0]
        send_ms = self.stages.get("send", [0])[0]
        if processing_ms <= 68 and scan_ms <= 100 and send_ms <= 68:
            return
        details = "; ".join(
            f"{name}={wall:.3f}/{cpu:.3f}ms(n={count})"
            for name, (wall, cpu, count) in self.stages.items()
        )
        logger.warning(
            "PLC_SLOW_CYCLE fifo=%s processing_ms=%.3f packets=%s "
            "stages=wall/thread_cpu (nested stages overlap): %s",
            fifo,
            processing_ms,
            self.packets,
            details,
        )
