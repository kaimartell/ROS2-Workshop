import ast
import glob
import logging
import re
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from host_agent.serial_repl import RawExecResult, SerialMicroPythonClient


USB_OK_MARKER = "__SPIKE_USB_OK__"
USB_ERR_MARKER = "__SPIKE_USB_ERR__"
USB_STATUS_MARKER = "__SPIKE_USB_STATUS__"
STOP_SPEED_EPSILON = 1e-3
SOUND_PROBE_CACHE_TTL_SEC = 10.0

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
        debug_repl_snippets: bool = False,
        **_: Any,
    ) -> None:
        self._serial_port_config = (serial_port or "auto").strip()
        self._baud = int(baud)
        self._motor_port = (motor_port or "A").strip().upper()
        self._prompt_timeout = max(0.5, float(prompt_timeout))
        self._command_timeout = max(1.0, float(command_timeout))
        self._sync_retries = max(2, int(sync_retries))
        self._debug_repl_snippets = bool(debug_repl_snippets)

        self._lock = threading.Lock()
        self._state = "idle"
        self._last_speed = 0.0
        self._timestamp = time.time()
        self._spike_connected = False
        self._last_error = ""
        self._resolved_port = ""
        self._sound_supported_cache: Optional[bool] = None
        self._sound_probe_monotonic = 0.0

    @staticmethod
    def _trim_text(text: str, limit: int = 800) -> str:
        normalized = (text or "").strip()
        if len(normalized) <= limit:
            return normalized
        return f"{normalized[:limit]}...(truncated)"

    @staticmethod
    def _clip_speed(speed: float) -> float:
        return max(-1.0, min(1.0, float(speed)))

    @staticmethod
    def _velocity_units(speed: float) -> int:
        return int(max(-1000, min(1000, round(float(speed) * 1000.0))))

    @staticmethod
    def _duty_units(speed: float) -> int:
        return int(max(-100, min(100, round(float(speed) * 100.0))))

    @staticmethod
    def _clamp_freq_hz(value: int) -> int:
        return int(max(50, min(5000, int(value))))

    @staticmethod
    def _clamp_duration_ms(value: int) -> int:
        return int(max(10, min(5000, int(value))))

    @staticmethod
    def _clamp_volume(value: int) -> int:
        return int(max(0, min(100, int(value))))

    @staticmethod
    def _prepare_snippet(lines: List[str]) -> str:
        return "\n".join(lines).rstrip("\n") + "\n"

    def _record_error(self, message: str) -> None:
        self._last_error = str(message)
        self._timestamp = time.time()

    def _normalize_port(self, port: str = "") -> str:
        candidate = str(port or "").strip().upper()
        if not candidate:
            candidate = self._motor_port
        if candidate not in {"A", "B", "C", "D", "E", "F"}:
            candidate = self._motor_port if self._motor_port in {"A", "B", "C", "D", "E", "F"} else "A"
        return candidate

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

        if result_error and "raw repl sync failed" in result_error.lower():
            return result_error
        if stderr.strip():
            return stderr.strip()
        if result_error:
            return result_error
        if stdout.strip() and USB_OK_MARKER not in stdout:
            return self._trim_text(stdout.strip(), limit=300)
        return default

    @staticmethod
    def _successful(result: RawExecResult) -> bool:
        return bool(result.ok and USB_OK_MARKER in (result.stdout or "") and not result.stderr.strip())

    def _build_core_snippet(self, *, port_letter: str, body_lines: List[str]) -> str:
        lines = [
            "try:",
            "    import motor",
            "except Exception as _e:",
            "    raise RuntimeError('Not running LEGO SPIKE firmware / knowledge base API. import motor failed: ' + repr(_e))",
            "",
            "try:",
            "    from hub import port",
            "except Exception as _e:",
            "    raise RuntimeError('Not running LEGO SPIKE firmware / knowledge base API. from hub import port failed: ' + repr(_e))",
            "",
            f"_letter = {port_letter!r}",
            "try:",
            "    _p = getattr(port, _letter)",
            "except Exception as _e:",
            "    _available = [_n for _n in dir(port) if len(_n) == 1 and 'A' <= _n <= 'Z']",
            "    raise RuntimeError('Invalid motor port ' + _letter + ' available=' + repr(_available) + ' err=' + repr(_e))",
            "",
        ]
        lines.extend(body_lines)
        lines.extend([
            "",
            f"print({USB_OK_MARKER!r})",
        ])

        wrapped: List[str] = ["try:"]
        for line in lines:
            if line:
                wrapped.append(f"    {line}")
            else:
                wrapped.append("")
        wrapped.extend(
            [
                "except Exception as _e:",
                f"    print({USB_ERR_MARKER!r} + ' ' + repr(_e))",
            ]
        )
        return self._prepare_snippet(wrapped)

    def _build_sound_snippet(self, body_lines: List[str]) -> str:
        lines = [
            "try:",
            "    from hub import sound",
            "except Exception as _e:",
            "    raise RuntimeError('Not running LEGO SPIKE firmware / knowledge base API. from hub import sound failed: ' + repr(_e))",
            "",
        ]
        lines.extend(body_lines)
        lines.extend(
            [
                "",
                f"print({USB_OK_MARKER!r})",
            ]
        )

        wrapped: List[str] = ["try:"]
        for line in lines:
            if line:
                wrapped.append(f"    {line}")
            else:
                wrapped.append("")
        wrapped.extend(
            [
                "except Exception as _e:",
                f"    print({USB_ERR_MARKER!r} + ' ' + repr(_e))",
            ]
        )
        return self._prepare_snippet(wrapped)

    def _stop_lines(self, stop_action: str) -> List[str]:
        normalized = str(stop_action or "coast").strip().lower()
        return [
            f"_stop_action = {normalized!r}",
            "try:",
            "    motor.stop(_p)",
            "except Exception as _e1:",
            "    try:",
            "        motor.run(_p, 0)",
            "    except Exception as _e2:",
            "        try:",
            "            motor.set_duty_cycle(_p, 0)",
            "        except Exception as _e3:",
            "            raise RuntimeError('stop failed for action=' + repr(_stop_action) + ': ' + repr(_e1) + '; ' + repr(_e2) + '; ' + repr(_e3))",
        ]

    def _set_idle(self) -> None:
        self._state = "idle"
        self._last_speed = 0.0
        self._timestamp = time.time()

    def _set_running(self, speed: float) -> None:
        self._state = "running"
        self._last_speed = float(speed)
        self._timestamp = time.time()

    def _probe_sound_supported(self, *, force: bool = False) -> bool:
        now = time.monotonic()
        if (
            (not force)
            and self._sound_supported_cache is not None
            and (now - self._sound_probe_monotonic) < SOUND_PROBE_CACHE_TTL_SEC
        ):
            return bool(self._sound_supported_cache)

        snippet = self._build_sound_snippet(
            body_lines=[
                "if not hasattr(sound, 'beep'):",
                "    raise RuntimeError('Missing hub.sound.beep API; firmware/API mismatch.')",
            ]
        )
        result = self._execute(snippet=snippet, timeout=max(2.0, self._prompt_timeout + 1.0))
        supported = self._successful(result)

        self._sound_probe_monotonic = now
        self._sound_supported_cache = supported
        return supported

    def _run_checked(
        self,
        *,
        snippet: str,
        timeout: float,
        success_payload: Dict[str, Any],
        failure_default: str,
        fail_key: str,
    ) -> Dict[str, Any]:
        result = self._execute(snippet=snippet, timeout=timeout)
        if self._successful(result):
            self._spike_connected = True
            self._last_error = ""
            return dict(success_payload)

        self._spike_connected = False
        error = self._extract_error_from_result(result, default=failure_default)
        self._record_error(error)
        logging.warning("[SPIKE_USB] %s failed: %s", fail_key, error)
        if self._debug_repl_snippets:
            logging.warning("[SPIKE_USB] failing snippet:\n%s", snippet)

        payload = {fail_key: False, "error": error}
        payload["stdout"] = self._trim_text(result.stdout)
        payload["stderr"] = self._trim_text(result.stderr)
        if self._debug_repl_snippets:
            payload["snippet"] = self._trim_text(snippet, limit=3000)
        return payload

    def run_with_diagnostics(self, speed: float, duration: float, port: str = "") -> Dict[str, Any]:
        with self._lock:
            safe_speed = self._clip_speed(speed)
            advisory_duration = max(0.0, float(duration))
            port_letter = self._normalize_port(port)
            velocity = self._velocity_units(safe_speed)

            snippet = self._build_core_snippet(
                port_letter=port_letter,
                body_lines=[
                    f"_velocity = {velocity}",
                    "motor.run(_p, _velocity)",
                ],
            )

            payload = self._run_checked(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
                success_payload={
                    "accepted": True,
                    "note": "nonblocking; duration handled by ROS stop commands",
                    "duration_advisory_sec": advisory_duration,
                    "port": port_letter,
                },
                failure_default="USB run command failed",
                fail_key="accepted",
            )
            if bool(payload.get("accepted", False)):
                if abs(safe_speed) < STOP_SPEED_EPSILON:
                    self._set_idle()
                else:
                    self._set_running(safe_speed)
            return payload

    def run(self, speed: float, duration: float, port: str = "") -> Dict[str, Any]:
        return self.run_with_diagnostics(speed=speed, duration=duration, port=port)

    def stop_with_diagnostics(self, port: str = "", stop_action: str = "coast") -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            snippet = self._build_core_snippet(
                port_letter=port_letter,
                body_lines=self._stop_lines(stop_action),
            )
            payload = self._run_checked(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
                success_payload={"stopped": True, "port": port_letter, "stop_action": stop_action},
                failure_default="USB stop command failed",
                fail_key="stopped",
            )
            self._set_idle()
            return payload

    def stop(self, port: str = "", stop_action: str = "coast") -> Dict[str, Any]:
        return self.stop_with_diagnostics(port=port, stop_action=stop_action)

    def run_for_degrees(
        self,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            safe_speed = self._clip_speed(speed)
            velocity = self._velocity_units(safe_speed)
            move_degrees = int(degrees)
            if move_degrees == 0:
                return {"accepted": False, "error": "degrees cannot be 0"}

            body_lines = [
                f"_velocity = {velocity}",
                f"_degrees = {move_degrees}",
                "if hasattr(motor, 'run_for_degrees'):",
                "    motor.run_for_degrees(_p, _degrees, _velocity)",
                "elif hasattr(motor, 'run_angle'):",
                "    motor.run_angle(_p, _velocity, _degrees)",
                "else:",
                "    raise RuntimeError('Missing motor.run_for_degrees API; firmware/API mismatch (also checked run_angle).')",
            ]
            body_lines.extend(self._stop_lines(stop_action))

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(3.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB run_for_degrees failed",
                fail_key="accepted",
            )
            self._set_idle()
            return payload

    def run_to_absolute(
        self,
        port: str,
        speed: float,
        position_degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            safe_speed = self._clip_speed(speed)
            velocity = self._velocity_units(safe_speed)
            target = int(position_degrees)

            body_lines = [
                f"_velocity = {velocity}",
                f"_target = {target}",
                "if hasattr(motor, 'run_to_absolute_position'):",
                "    motor.run_to_absolute_position(_p, _target, _velocity)",
                "elif hasattr(motor, 'run_to_position'):",
                "    motor.run_to_position(_p, _target, _velocity)",
                "elif hasattr(motor, 'run_target'):",
                "    motor.run_target(_p, _velocity, _target)",
                "else:",
                "    raise RuntimeError('Missing motor.run_to_absolute_position API; firmware/API mismatch (checked run_to_position/run_target).')",
            ]
            body_lines.extend(self._stop_lines(stop_action))

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(3.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB run_to_absolute failed",
                fail_key="accepted",
            )
            self._set_idle()
            return payload

    def run_to_relative(
        self,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            safe_speed = self._clip_speed(speed)
            velocity = self._velocity_units(safe_speed)
            delta = int(degrees)
            if delta == 0:
                return {"accepted": False, "error": "degrees cannot be 0"}

            body_lines = [
                f"_velocity = {velocity}",
                f"_delta = {delta}",
                "if hasattr(motor, 'run_to_relative_position'):",
                "    motor.run_to_relative_position(_p, _delta, _velocity)",
                "elif hasattr(motor, 'run_for_degrees'):",
                "    motor.run_for_degrees(_p, _delta, _velocity)",
                "elif hasattr(motor, 'run_angle'):",
                "    motor.run_angle(_p, _velocity, _delta)",
                "else:",
                "    raise RuntimeError('Missing motor.run_to_relative_position API; firmware/API mismatch (checked run_for_degrees/run_angle).')",
            ]
            body_lines.extend(self._stop_lines(stop_action))

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(3.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB run_to_relative failed",
                fail_key="accepted",
            )
            self._set_idle()
            return payload

    def reset_relative(self, port: str = "A") -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            body_lines = [
                "if hasattr(motor, 'reset_relative_position'):",
                "    try:",
                "        motor.reset_relative_position(_p, 0)",
                "    except TypeError:",
                "        motor.reset_relative_position(_p)",
                "elif hasattr(motor, 'set_relative_position'):",
                "    motor.set_relative_position(_p, 0)",
                "else:",
                "    raise RuntimeError('Missing motor.reset_relative_position API; firmware/API mismatch.')",
            ]

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(2.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB reset_relative failed",
                fail_key="accepted",
            )
            self._set_idle()
            return payload

    def set_duty_cycle(self, port: str, speed: float) -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            duty = self._duty_units(speed)

            body_lines = [
                f"_duty = {duty}",
                "if hasattr(motor, 'set_duty_cycle'):",
                "    motor.set_duty_cycle(_p, _duty)",
                "elif hasattr(motor, 'run'):",
                "    motor.run(_p, int(_duty * 10))",
                "else:",
                "    raise RuntimeError('Missing motor.set_duty_cycle API; firmware/API mismatch.')",
            ]
            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(2.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB set_duty_cycle failed",
                fail_key="accepted",
            )
            if bool(payload.get("accepted", False)):
                if abs(duty) <= 0:
                    self._set_idle()
                else:
                    self._set_running(float(duty) / 100.0)
            return payload

    def sound_beep(
        self,
        freq_hz: int,
        duration_ms: int,
        volume: int,
    ) -> Dict[str, Any]:
        with self._lock:
            safe_freq = self._clamp_freq_hz(freq_hz)
            safe_duration = self._clamp_duration_ms(duration_ms)
            safe_volume = self._clamp_volume(volume)

            snippet = self._build_sound_snippet(
                body_lines=[
                    "if not hasattr(sound, 'beep'):",
                    "    raise RuntimeError('Missing hub.sound.beep API; firmware/API mismatch.')",
                    f"sound.beep({safe_freq}, {safe_duration}, {safe_volume})",
                ]
            )
            payload = self._run_checked(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
                success_payload={
                    "accepted": True,
                    "freq_hz": safe_freq,
                    "duration_ms": safe_duration,
                    "volume": safe_volume,
                },
                failure_default="USB sound_beep failed",
                fail_key="accepted",
            )
            if bool(payload.get("accepted", False)):
                self._sound_supported_cache = True
                self._sound_probe_monotonic = time.monotonic()
            else:
                self._sound_supported_cache = False
            payload.setdefault("freq_hz", safe_freq)
            payload.setdefault("duration_ms", safe_duration)
            payload.setdefault("volume", safe_volume)
            return payload

    def sound_stop(self) -> Dict[str, Any]:
        with self._lock:
            snippet = self._build_sound_snippet(
                body_lines=[
                    "if hasattr(sound, 'stop'):",
                    "    sound.stop()",
                    "elif hasattr(sound, 'beep'):",
                    "    pass",
                    "else:",
                    "    raise RuntimeError('Missing hub.sound API; firmware/API mismatch.')",
                ]
            )
            payload = self._run_checked(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
                success_payload={"stopped": True},
                failure_default="USB sound_stop failed",
                fail_key="stopped",
            )
            if bool(payload.get("stopped", False)):
                self._sound_supported_cache = True
                self._sound_probe_monotonic = time.monotonic()
            return payload

    def motor_status(self, port: str = "A") -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            body_lines = [
                "_status = {}",
                "_status['port'] = _letter",
                "if hasattr(motor, 'relative_position'):",
                "    try:",
                "        _status['relative_position'] = motor.relative_position(_p)",
                "    except Exception:",
                "        pass",
                "if hasattr(motor, 'absolute_position'):",
                "    try:",
                "        _status['absolute_position'] = motor.absolute_position(_p)",
                "    except Exception:",
                "        pass",
                "if hasattr(motor, 'velocity'):",
                "    try:",
                "        _status['velocity'] = motor.velocity(_p)",
                "    except Exception:",
                "        pass",
                f"print({USB_STATUS_MARKER!r} + repr(_status))",
            ]
            snippet = self._build_core_snippet(port_letter=port_letter, body_lines=body_lines)
            result = self._execute(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
            )
            if not self._successful(result):
                self._spike_connected = False
                error = self._extract_error_from_result(result, default="USB motor_status failed")
                self._record_error(error)
                payload: Dict[str, Any] = {"ok": False, "error": error}
                if self._debug_repl_snippets:
                    payload["snippet"] = self._trim_text(snippet, limit=3000)
                    payload["stdout"] = self._trim_text(result.stdout)
                    payload["stderr"] = self._trim_text(result.stderr)
                return payload

            status_payload: Dict[str, Any] = {}
            for line in (result.stdout or "").splitlines():
                if USB_STATUS_MARKER in line:
                    raw = line.split(USB_STATUS_MARKER, 1)[1].strip()
                    try:
                        parsed = ast.literal_eval(raw)
                        if isinstance(parsed, dict):
                            status_payload = parsed
                    except Exception:  # noqa: BLE001
                        status_payload = {}
                    break

            self._spike_connected = True
            self._last_error = ""
            return {
                "ok": True,
                "state": self._state,
                "last_speed": self._last_speed,
                "timestamp": self._timestamp,
                **status_payload,
            }

    def _minimal_repl_check(self) -> Tuple[bool, RawExecResult]:
        result = self._execute(
            snippet=f"print({USB_OK_MARKER!r})\n",
            timeout=max(self._prompt_timeout + 1.0, 2.0),
        )
        repl_ok = self._successful(result)
        return repl_ok, result

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "state": self._state,
                "last_speed": self._last_speed,
                "timestamp": self._timestamp,
            }

    def health(self) -> Dict[str, Any]:
        with self._lock:
            repl_ok, result = self._minimal_repl_check()
            self._spike_connected = repl_ok
            sound_supported = repl_ok and self._probe_sound_supported(force=False)

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
                "sound_supported": bool(sound_supported),
            }
            if self._resolved_port:
                payload["serial_port"] = self._resolved_port
            if self._last_error:
                payload["detail"] = self._last_error
            if self._debug_repl_snippets and (not repl_ok):
                payload["stdout"] = self._trim_text(result.stdout)
                payload["stderr"] = self._trim_text(result.stderr)
            return payload

    def close(self) -> None:
        try:
            self.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.sound_stop()
        except Exception:  # noqa: BLE001
            pass
