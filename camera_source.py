"""Single latest-frame camera entry shared by all present and future vision."""

import queue
import threading
import time
from typing import Optional
from models import FramePacket

class LatestFrameSource:
    def __init__(self, camera, strategy: str, read_timeout: float) -> None:
        self._camera = camera
        self._strategy = strategy
        self._read_timeout = read_timeout
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest: Optional[FramePacket] = None
        self._failure: Optional[BaseException] = None

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def _run(self) -> None:
        sequence = 0
        while not self._stop.is_set():
            try:
                frame = self._camera.read_cv2_image(
                    strategy=self._strategy,
                    timeout=self._read_timeout,
                )
            except queue.Empty:
                continue
            except BaseException as error:
                with self._condition:
                    self._failure = error
                    self._condition.notify_all()
                return
            if frame is None:
                continue
            sequence += 1
            packet = FramePacket(frame, sequence, time.monotonic())
            with self._condition:
                self._latest = packet
                self._condition.notify_all()

    def wait_after(self, sequence: int, timeout: float) -> Optional[FramePacket]:
        with self._condition:
            if (self._latest is None or self._latest.sequence <= sequence) and self._failure is None:
                self._condition.wait(max(0.0, timeout))
            if self._failure is not None:
                raise self._failure
            if self._latest is None or self._latest.sequence <= sequence:
                return None
            return self._latest

    def age(self, now: Optional[float] = None) -> float:
        timestamp = time.monotonic() if now is None else now
        with self._condition:
            packet = self._latest
        return float("inf") if packet is None else max(0.0, timestamp - packet.captured_at)


    def close(self) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(0.5)
