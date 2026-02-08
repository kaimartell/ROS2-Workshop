import logging
import threading
import time
from typing import Any, Dict


class MockBackend:
    """Mock backend that simulates a motor run window and never hard-fails."""

    name = "mock"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._running_until = 0.0
        self._last_speed = 0.0
        self._timestamp = time.time()

    def run(self, speed: float, duration: float) -> Dict[str, Any]:
        with self._lock:
            safe_speed = float(speed)
            safe_duration = max(0.0, float(duration))
            self._last_speed = safe_speed
            self._running_until = time.monotonic() + safe_duration
            self._timestamp = time.time()

        logging.info("[MOCK] motor/run speed=%.3f duration=%.3f", safe_speed, safe_duration)
        return {"accepted": True}

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            self._running_until = 0.0
            self._last_speed = 0.0
            self._timestamp = time.time()

        logging.info("[MOCK] motor/stop")
        return {"stopped": True}

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            running = time.monotonic() < self._running_until
            state = "running" if running else "idle"
            return {
                "state": state,
                "last_speed": self._last_speed,
                "timestamp": self._timestamp,
            }

    def health(self) -> Dict[str, Any]:
        return {"ok": True, "backend": self.name, "spike_connected": True}

    def close(self) -> None:
        self.stop()
