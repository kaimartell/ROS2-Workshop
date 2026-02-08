import json
import threading
import time
from typing import Any, Dict, Optional, Tuple
from urllib import error, request


class HostAgentHttpClient:
    """Small HTTP client for host-agent communication."""

    def __init__(self, base_url: str, timeout: float = 1.5) -> None:
        self._base_url = base_url.rstrip("/")
        self._timeout = max(0.1, float(timeout))
        self._lock = threading.Lock()
        self._last_error: str = ""
        self._last_health_latency_ms: Optional[float] = None

    def set_base_url(self, base_url: str) -> None:
        self._base_url = base_url.rstrip("/")

    def set_timeout(self, timeout: float) -> None:
        self._timeout = max(0.1, float(timeout))

    def get_base_url(self) -> str:
        return self._base_url

    def get_last_error(self) -> str:
        with self._lock:
            return self._last_error

    def get_last_health_latency_ms(self) -> Optional[float]:
        with self._lock:
            return self._last_health_latency_ms

    def run_motor(self, speed: float, duration: float) -> Dict[str, Any]:
        response, _meta = self.run_motor_with_meta(speed=speed, duration=duration)
        return response

    def run_motor_with_meta(self, speed: float, duration: float) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload = {"speed": float(speed), "duration": max(0.0, float(duration))}
        response, meta = self._request("POST", "/motor/run", payload)
        self._record_error(meta.get("error", ""))
        if response is None:
            return {"accepted": False, "error": meta.get("error", "unreachable")}, meta
        return response, meta

    def stop_motor(self) -> Dict[str, Any]:
        response, _meta = self.stop_motor_with_meta()
        return response

    def stop_motor_with_meta(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        response, meta = self._request("POST", "/motor/stop", {})
        self._record_error(meta.get("error", ""))
        if response is None:
            return {"stopped": False, "error": meta.get("error", "unreachable")}, meta
        return response, meta

    def get_state(self) -> Dict[str, Any]:
        response, _meta = self.get_state_with_meta()
        return response

    def get_state_with_meta(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        response, meta = self._request("GET", "/state", None)
        self._record_error(meta.get("error", ""))
        if response is None:
            return {"state": "unreachable", "last_speed": 0.0}, meta
        return response, meta

    def get_health(self) -> Dict[str, Any]:
        response, _meta = self.get_health_with_meta()
        return response

    def get_health_with_meta(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        response, meta = self._request("GET", "/health", None)
        self._record_error(meta.get("error", ""))
        with self._lock:
            self._last_health_latency_ms = meta.get("latency_ms")
        if response is None:
            return {"ok": False, "backend": "unreachable", "spike_connected": False}, meta
        return response, meta

    def _record_error(self, message: str) -> None:
        with self._lock:
            self._last_error = str(message or "")

    def _request(
        self, method: str, path: str, payload: Optional[Dict[str, Any]]
    ) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
        start = time.perf_counter()
        url = f"{self._base_url}{path}"
        headers = {"Accept": "application/json"}
        body = None

        if payload is not None:
            headers["Content-Type"] = "application/json"
            body = json.dumps(payload).encode("utf-8")

        req = request.Request(url=url, data=body, method=method, headers=headers)
        try:
            with request.urlopen(req, timeout=self._timeout) as resp:
                raw = resp.read().decode("utf-8")
            meta = {
                "ok": True,
                "latency_ms": (time.perf_counter() - start) * 1000.0,
                "error": "",
            }
        except error.HTTPError as exc:
            raw = ""
            try:
                raw = exc.read().decode("utf-8")
            except Exception:  # noqa: BLE001
                raw = ""
            meta = {
                "ok": True,
                "latency_ms": (time.perf_counter() - start) * 1000.0,
                "error": f"HTTP {exc.code} {exc.reason}",
            }
            if not raw:
                return {"error": meta["error"]}, meta
        except (error.URLError, TimeoutError, ValueError) as exc:
            meta = {
                "ok": False,
                "latency_ms": (time.perf_counter() - start) * 1000.0,
                "error": str(exc) or exc.__class__.__name__,
            }
            return None, meta

        if not raw:
            return {}, meta

        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return {"raw": raw}, meta

        if isinstance(parsed, dict):
            return parsed, meta
        return {"raw": parsed}, meta
