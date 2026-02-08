import glob
import logging
import re
import threading
import textwrap
import time
from typing import Any, Dict, List, Optional, Tuple

from host_agent.serial_repl import RawExecResult, SerialMicroPythonClient


USB_OK_MARKER = "__SPIKE_USB_OK__"
USB_ERR_MARKER = "__SPIKE_USB_ERR__"

_SERIAL_CANDIDATE_REGEX = re.compile(
    r"^/dev/(cu|tty)\.(usbmodem|usbserial|SLAB_USBtoUART|wchusbserial|ttyUSB|ttyACM)",
    re.IGNORECASE,
)


def _is_serial_candidate(device: str, description: str = "", hwid: str = "") -> bool:
    if _SERIAL_CANDIDATE_REGEX.match(device):
        return True

    lowered = f"{device} {description} {hwid}".lower()
    return any(
        token in lowered
        for token in ("usb", "modem", "serial", "lego", "spike", "cp210", "acm", "uart")
    )


def list_serial_ports() -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    seen: set[str] = set()

    try:
        from serial.tools import list_ports  # type: ignore

        for port in list_ports.comports():
            device = str(port.device)
            if not device or device in seen:
                continue
            if not _is_serial_candidate(
                device=device,
                description=str(getattr(port, "description", "")),
                hwid=str(getattr(port, "hwid", "")),
            ):
                continue
            seen.add(device)
            rows.append(
                {
                    "device": device,
                    "description": str(getattr(port, "description", "")),
                    "hwid": str(getattr(port, "hwid", "")),
                }
            )
    except Exception:  # noqa: BLE001
        pass

    for pattern in (
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.SLAB_USBtoUART*",
        "/dev/cu.wchusbserial*",
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/tty.SLAB_USBtoUART*",
    ):
        for device in glob.glob(pattern):
            if device in seen:
                continue
            seen.add(device)
            rows.append({"device": device, "description": "", "hwid": ""})

    rows.sort(key=lambda row: row.get("device", ""))
    return rows


class SpikeUsbBackend:
    """SPIKE backend via USB serial RAW REPL execution."""

    name = "spike_usb"

    def __init__(
        self,
        serial_port: str = "auto",
        baud: int = 115200,
        motor_port: str = "A",
        prompt_timeout: float = 2.0,
        command_timeout: float = 8.0,
        sync_retries: int = 2,
        **_: Any,
    ) -> None:
        self._serial_port_config = (serial_port or "auto").strip()
        self._baud = int(baud)
        self._motor_port = (motor_port or "A").strip().upper()
        self._prompt_timeout = max(0.5, float(prompt_timeout))
        self._command_timeout = max(1.0, float(command_timeout))
        self._sync_retries = max(2, int(sync_retries))

        self._lock = threading.Lock()
        self._state = "idle"
        self._last_speed = 0.0
        self._running_until = 0.0
        self._timestamp = time.time()
        self._spike_connected = False
        self._last_error = ""
        self._resolved_port = ""

    def _record_error(self, message: str) -> None:
        self._last_error = message
        self._timestamp = time.time()

    def _resolve_serial_port(self) -> Tuple[Optional[str], str]:
        configured = self._serial_port_config
        if configured and configured.lower() != "auto":
            self._resolved_port = configured
            return configured, ""

        candidates = list_serial_ports()
        devices = [row["device"] for row in candidates if row.get("device")]

        if len(devices) == 1:
            self._resolved_port = devices[0]
            return devices[0], ""

        if not devices:
            return None, "No USB serial candidates found. Use --serial-port /dev/cu.usbmodemXXXX."

        joined = ", ".join(devices)
        return (
            None,
            f"Multiple USB serial candidates found ({joined}). Use --serial-port to select one.",
        )

    def _trim_text(self, text: str, limit: int = 800) -> str:
        normalized = (text or "").strip()
        if len(normalized) <= limit:
            return normalized
        return f"{normalized[:limit]}...(truncated)"

    def _execute(self, snippet: str, timeout: Optional[float] = None) -> RawExecResult:
        port, error = self._resolve_serial_port()
        if port is None:
            return RawExecResult(ok=False, error=error)

        try:
            with SerialMicroPythonClient(
                port=port,
                baud=self._baud,
                prompt_timeout=self._prompt_timeout,
                exec_timeout=self._command_timeout,
                sync_retries=self._sync_retries,
            ) as client:
                result = client.execute_raw(snippet, timeout=timeout)
        except Exception as exc:  # noqa: BLE001
            result = RawExecResult(ok=False, error=str(exc))

        if not result.ok and "raw repl sync failed" in (result.error or "").lower():
            result.error = (
                "raw repl sync failed; reset hub and close other apps that may hold the serial port"
            )

        return result

    def _extract_error_from_result(self, result: RawExecResult, default: str) -> str:
        stdout = result.stdout or ""
        stderr = result.stderr or ""
        result_error = (result.error or "").strip()

        marker_index = stdout.find(USB_ERR_MARKER)
        if marker_index >= 0:
            detail = stdout[marker_index + len(USB_ERR_MARKER) :].strip(" :\n\r\t")
            if detail:
                return detail

        # Preserve explicit raw REPL sync diagnostics for caller-visible API errors.
        if result_error and "raw repl sync failed" in result_error.lower():
            return result_error

        if stderr.strip():
            return stderr.strip()

        if result_error:
            return result_error

        if stdout.strip() and USB_OK_MARKER not in stdout:
            return self._trim_text(stdout.strip(), limit=300)

        return default

    def _build_run_snippet(self, speed: float, duration: float) -> str:
        velocity = int(max(-1000, min(1000, int(float(speed) * 1000.0))))
        duration = max(0.0, float(duration))
        port_letter = self._motor_port

        snippet = f"""
import time
try:
    import motor
except Exception as _e:
    raise RuntimeError("Not running LEGO SPIKE firmware / knowledge base API. import motor failed: " + repr(_e))

try:
    from hub import port
except Exception as _e:
    raise RuntimeError("Not running LEGO SPIKE firmware / knowledge base API. from hub import port failed: " + repr(_e))

_letter = {port_letter!r}
_available = []
for _name in dir(port):
    if len(_name) == 1 and _name >= "A" and _name <= "Z":
        _available.append(_name)
if _letter not in _available:
    raise RuntimeError("Invalid motor port " + _letter + " available=" + repr(_available))

_p = getattr(port, _letter)

try:
    motor.run(_p, {velocity})
except Exception as _e:
    raise RuntimeError("motor.run failed: " + repr(_e))

time.sleep({duration})
try:
    motor.stop(_p)
except Exception as _e1:
    try:
        motor.run(_p, 0)
    except Exception as _e2:
        try:
            motor.set_duty_cycle(_p, 0)
        except Exception as _e3:
            raise RuntimeError(
                "motor stop fallbacks failed: "
                + repr(_e1)
                + "; "
                + repr(_e2)
                + "; "
                + repr(_e3)
            )

print({USB_OK_MARKER!r})
"""
        return textwrap.dedent(snippet).strip() + "\n"

    def _build_stop_snippet(self) -> str:
        port_letter = self._motor_port
        snippet = f"""
try:
    import motor
except Exception as _e:
    raise RuntimeError("Not running LEGO SPIKE firmware / knowledge base API. import motor failed: " + repr(_e))

try:
    from hub import port
except Exception as _e:
    raise RuntimeError("Not running LEGO SPIKE firmware / knowledge base API. from hub import port failed: " + repr(_e))

_letter = {port_letter!r}
_available = []
for _name in dir(port):
    if len(_name) == 1 and _name >= "A" and _name <= "Z":
        _available.append(_name)
if _letter not in _available:
    raise RuntimeError("Invalid motor port " + _letter + " available=" + repr(_available))

_p = getattr(port, _letter)
try:
    motor.stop(_p)
except Exception as _e1:
    try:
        motor.run(_p, 0)
    except Exception as _e2:
        try:
            motor.set_duty_cycle(_p, 0)
        except Exception as _e3:
            raise RuntimeError(
                "motor stop fallbacks failed: "
                + repr(_e1)
                + "; "
                + repr(_e2)
                + "; "
                + repr(_e3)
            )

print({USB_OK_MARKER!r})
"""
        return textwrap.dedent(snippet).strip() + "\n"

    def _set_idle(self) -> None:
        self._state = "idle"
        self._last_speed = 0.0
        self._running_until = 0.0
        self._timestamp = time.time()

    def run_with_diagnostics(self, speed: float, duration: float) -> Dict[str, Any]:
        with self._lock:
            safe_speed = max(-1.0, min(1.0, float(speed)))
            safe_duration = max(0.0, float(duration))
            self._state = "running"
            self._last_speed = safe_speed
            self._running_until = time.monotonic() + safe_duration
            self._timestamp = time.time()

            result = self._execute(
                snippet=self._build_run_snippet(safe_speed, safe_duration),
                timeout=self._command_timeout + safe_duration + 1.0,
            )

            self._set_idle()

            accepted = bool(result.ok and USB_OK_MARKER in (result.stdout or "") and not result.stderr.strip())
            if accepted:
                self._spike_connected = True
                self._last_error = ""
                return {"accepted": True}

            self._spike_connected = False
            error = self._extract_error_from_result(result, default="USB run command failed")
            self._record_error(error)
            logging.warning("[SPIKE_USB] motor/run failed: %s", error)
            return {
                "accepted": False,
                "error": error,
                "stdout": self._trim_text(result.stdout),
                "stderr": self._trim_text(result.stderr),
            }

    def run(self, speed: float, duration: float) -> Dict[str, Any]:
        return self.run_with_diagnostics(speed=speed, duration=duration)

    def stop_with_diagnostics(self) -> Dict[str, Any]:
        with self._lock:
            result = self._execute(
                snippet=self._build_stop_snippet(),
                timeout=max(2.0, self._command_timeout),
            )

            self._set_idle()

            stopped = bool(result.ok and USB_OK_MARKER in (result.stdout or "") and not result.stderr.strip())
            if stopped:
                self._spike_connected = True
                self._last_error = ""
                return {"stopped": True}

            self._spike_connected = False
            error = self._extract_error_from_result(result, default="USB stop command failed")
            self._record_error(error)
            logging.warning("[SPIKE_USB] motor/stop failed: %s", error)
            return {
                "stopped": False,
                "error": error,
                "stdout": self._trim_text(result.stdout),
                "stderr": self._trim_text(result.stderr),
            }

    def stop(self) -> Dict[str, Any]:
        return self.stop_with_diagnostics()

    def _minimal_repl_check(self) -> Tuple[bool, RawExecResult]:
        result = self._execute(
            snippet=f"print({USB_OK_MARKER!r})",
            timeout=max(self._prompt_timeout + 1.0, 2.0),
        )
        repl_ok = bool(result.ok and USB_OK_MARKER in (result.stdout or "") and not result.stderr.strip())
        return repl_ok, result

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            if self._state == "running" and time.monotonic() >= self._running_until:
                self._set_idle()

            return {
                "state": self._state,
                "last_speed": self._last_speed,
                "timestamp": self._timestamp,
            }

    def health(self) -> Dict[str, Any]:
        with self._lock:
            repl_ok, result = self._minimal_repl_check()
            self._spike_connected = repl_ok

            if repl_ok:
                self._last_error = ""
            else:
                error = self._extract_error_from_result(result, default="raw repl health check failed")
                self._record_error(error)

            payload: Dict[str, Any] = {
                "ok": True,
                "backend": self.name,
                "spike_connected": repl_ok,
                "repl_ok": repl_ok,
            }
            if self._resolved_port:
                payload["serial_port"] = self._resolved_port
            if self._last_error:
                payload["detail"] = self._last_error
            return payload

    def close(self) -> None:
        with self._lock:
            self._set_idle()
