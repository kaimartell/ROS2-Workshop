import json
import threading
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from spike_workshop_hw.http_client import HostAgentHttpClient


class SpikeHwClientNode(Node):
    def __init__(self) -> None:
        super().__init__("spike_hw_client_node")

        self.declare_parameter("host_agent_url", "http://host.docker.internal:8000")
        self.declare_parameter("poll_hz", 2.0)
        self.declare_parameter("http_timeout_sec", 1.5)
        self.declare_parameter("unreachable_log_throttle_sec", 10.0)
        self.declare_parameter("consecutive_failures_to_mark_down", 2)
        self.declare_parameter("consecutive_successes_to_mark_up", 1)

        host_agent_url = str(self.get_parameter("host_agent_url").value).rstrip("/")
        poll_hz = float(self.get_parameter("poll_hz").value)
        poll_hz = poll_hz if poll_hz > 0.0 else 2.0
        http_timeout_sec = float(self.get_parameter("http_timeout_sec").value)
        http_timeout_sec = http_timeout_sec if http_timeout_sec > 0.0 else 1.5
        unreachable_log_throttle_sec = float(
            self.get_parameter("unreachable_log_throttle_sec").value
        )
        unreachable_log_throttle_sec = max(0.0, unreachable_log_throttle_sec)
        consecutive_failures_to_mark_down = int(
            self.get_parameter("consecutive_failures_to_mark_down").value
        )
        consecutive_failures_to_mark_down = max(1, consecutive_failures_to_mark_down)
        consecutive_successes_to_mark_up = int(
            self.get_parameter("consecutive_successes_to_mark_up").value
        )
        consecutive_successes_to_mark_up = max(1, consecutive_successes_to_mark_up)

        self._host_agent_url = host_agent_url
        self._http_timeout_sec = http_timeout_sec
        self._unreachable_log_throttle_sec = unreachable_log_throttle_sec
        self._consecutive_failures_to_mark_down = consecutive_failures_to_mark_down
        self._consecutive_successes_to_mark_up = consecutive_successes_to_mark_up

        self._http_client = HostAgentHttpClient(host_agent_url, timeout=http_timeout_sec)

        self._conn_lock = threading.Lock()
        self._connection_state = "unknown"
        self._last_error = ""
        self._last_health_latency_ms: Optional[float] = None
        self._last_latency_ms: Optional[float] = None
        self._consecutive_failures = 0
        self._consecutive_successes = 0
        self._last_unreachable_log_monotonic = 0.0

        self._cmd_lock = threading.Lock()
        self._pending_cmd: Optional[Tuple[float, float]] = None
        self._dropped_cmd_count = 0
        self._cmd_event = threading.Event()

        self._shutting_down = False
        self._cmd_worker_thread = threading.Thread(
            target=self._command_worker_loop,
            name="spike_hw_cmd_worker",
            daemon=True,
        )
        self._cmd_worker_thread.start()

        self._state_pub = self.create_publisher(String, "/spike/state", 10)
        self._cmd_sub = self.create_subscription(String, "/spike/cmd", self._on_cmd, 10)
        self._ping_srv = self.create_service(Trigger, "/spike/ping", self._on_ping)
        self._poll_timer = self.create_timer(1.0 / poll_hz, self._poll_state)

        self._safe_log(
            "info",
            (
                "spike_hw_client_node online: "
                f"host_agent_url={host_agent_url}, poll_hz={poll_hz}, "
                f"http_timeout_sec={http_timeout_sec:.2f}, "
                f"unreachable_log_throttle_sec={unreachable_log_throttle_sec:.1f}, "
                f"consecutive_failures_to_mark_down={consecutive_failures_to_mark_down}, "
                f"consecutive_successes_to_mark_up={consecutive_successes_to_mark_up}"
            ),
        )
        self._safe_log("info", "Ping service ready on /spike/ping")

    def _context_ok(self) -> bool:
        try:
            return rclpy.ok(context=self.context)
        except Exception:  # noqa: BLE001
            return False

    def _safe_log(self, level: str, message: str) -> None:
        if not self._context_ok():
            return
        logger = self.get_logger()
        if level == "warning":
            logger.warning(message)
        elif level == "error":
            logger.error(message)
        else:
            logger.info(message)

    @staticmethod
    def _to_float(value: Any) -> Optional[float]:
        try:
            if value is None:
                return None
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _fmt_latency_ms(value: Optional[float]) -> str:
        if value is None:
            return "n/a"
        return f"{value:.1f}"

    def _record_connectivity(
        self,
        *,
        transport_ok: bool,
        source: str,
        latency_ms: Optional[float],
        error: str,
    ) -> None:
        event = None
        now = time.monotonic()
        normalized_error = str(error or "").strip()

        with self._conn_lock:
            self._last_latency_ms = latency_ms

            if transport_ok:
                self._consecutive_failures = 0
                self._consecutive_successes += 1
                self._last_error = ""

                if (
                    self._connection_state != "connected"
                    and self._consecutive_successes >= self._consecutive_successes_to_mark_up
                ):
                    self._connection_state = "connected"
                    event = (
                        "info",
                        (
                            f"Connected to host agent at {self._host_agent_url} "
                            f"(source={source}, latency_ms={self._fmt_latency_ms(latency_ms)})."
                        ),
                    )
                return

            self._consecutive_successes = 0
            self._consecutive_failures += 1
            if normalized_error:
                self._last_error = normalized_error
            elif not self._last_error:
                self._last_error = "request failed"

            details = (
                f"source={source}, timeout_sec={self._http_timeout_sec:.2f}, "
                f"latency_ms={self._fmt_latency_ms(latency_ms)}, error={self._last_error}"
            )

            if (
                self._connection_state != "unreachable"
                and self._consecutive_failures >= self._consecutive_failures_to_mark_down
            ):
                self._connection_state = "unreachable"
                self._last_unreachable_log_monotonic = now
                event = (
                    "warning",
                    (
                        f"Host agent unreachable at {self._host_agent_url} after "
                        f"{self._consecutive_failures} consecutive transport failures ({details})."
                    ),
                )
            elif self._connection_state == "unreachable":
                throttled_for = now - self._last_unreachable_log_monotonic
                if (
                    self._unreachable_log_throttle_sec == 0.0
                    or throttled_for >= self._unreachable_log_throttle_sec
                ):
                    self._last_unreachable_log_monotonic = now
                    event = (
                        "warning",
                        (
                            f"Host agent still unreachable at {self._host_agent_url} "
                            f"({details})."
                        ),
                    )

        if event is not None:
            level, message = event
            self._safe_log(level, message)

    def _queue_latest_command(self, speed: float, duration: float) -> None:
        with self._cmd_lock:
            if self._pending_cmd is not None:
                self._dropped_cmd_count += 1
            self._pending_cmd = (speed, duration)
            self._cmd_event.set()

    def _pop_latest_command(self) -> Tuple[Optional[Tuple[float, float]], int]:
        with self._cmd_lock:
            cmd = self._pending_cmd
            dropped = self._dropped_cmd_count
            self._pending_cmd = None
            self._dropped_cmd_count = 0
            if self._pending_cmd is None:
                self._cmd_event.clear()
            return cmd, dropped

    def _command_worker_loop(self) -> None:
        while True:
            self._cmd_event.wait(timeout=0.2)
            if self._shutting_down:
                return

            cmd, dropped = self._pop_latest_command()
            if cmd is None:
                continue

            speed, duration = cmd
            result, meta = self._http_client.run_motor_with_meta(speed=speed, duration=duration)
            self._record_connectivity(
                transport_ok=bool(meta.get("ok", False)),
                source="POST /motor/run",
                latency_ms=self._to_float(meta.get("latency_ms")),
                error=str(meta.get("error", "")),
            )

            accepted = bool(result.get("accepted", False))
            if accepted:
                dropped_note = ""
                if dropped > 0:
                    dropped_note = f" (dropped_intermediate={dropped})"
                self._safe_log(
                    "info",
                    (
                        f"Forwarded motor command speed={speed:.3f} "
                        f"duration={duration:.3f}s{dropped_note}"
                    ),
                )
            else:
                self._safe_log("warning", f"Host agent did not accept command: {result}")

    def _on_cmd(self, msg: String) -> None:
        if self._shutting_down:
            return

        try:
            payload = json.loads(msg.data)
            speed = float(payload.get("speed", 0.0))
            duration = float(payload.get("duration", 0.0))
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._safe_log("warning", f"Ignoring malformed /spike/cmd payload: {exc}")
            return

        self._queue_latest_command(speed=speed, duration=duration)

    def _poll_state(self) -> None:
        if self._shutting_down:
            return
        state, meta = self._http_client.get_state_with_meta()
        self._record_connectivity(
            transport_ok=bool(meta.get("ok", False)),
            source="GET /state",
            latency_ms=self._to_float(meta.get("latency_ms")),
            error=str(meta.get("error", "")),
        )
        self._publish_state(state)

    def _publish_state(self, state: Dict[str, Any]) -> None:
        if not self._context_ok():
            return
        msg = String()
        msg.data = json.dumps(state, separators=(",", ":"), sort_keys=True)
        try:
            self._state_pub.publish(msg)
        except Exception:  # noqa: BLE001
            return

    def _on_ping(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        health, meta = self._http_client.get_health_with_meta()
        health_latency_ms = self._to_float(meta.get("latency_ms"))
        self._last_health_latency_ms = health_latency_ms

        self._record_connectivity(
            transport_ok=bool(meta.get("ok", False)),
            source="GET /health",
            latency_ms=health_latency_ms,
            error=str(meta.get("error", "")),
        )

        ok = bool(health.get("ok", False))
        backend = str(health.get("backend", "unknown"))
        spike_connected = bool(health.get("spike_connected", False))

        requires_connection = backend not in {"mock", "unknown", "unreachable"}
        success = ok and (spike_connected or not requires_connection)

        with self._conn_lock:
            connection_state = self._connection_state
            last_error = self._last_error

        response.success = success
        response.message = json.dumps(
            {
                "ok": ok,
                "backend": backend,
                "spike_connected": spike_connected,
                "requires_connection": requires_connection,
                "host_agent_url": self._host_agent_url,
                "connection_state": connection_state,
                "last_health_latency_ms": health_latency_ms,
                "last_error": last_error,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        return response

    def stop_motor(self) -> Dict[str, Any]:
        result = self._http_client.stop_motor()
        return result

    def request_shutdown_cleanup(self) -> None:
        self._shutting_down = True
        if self._poll_timer is not None:
            self._poll_timer.cancel()

        self._cmd_event.set()
        if self._cmd_worker_thread.is_alive():
            self._cmd_worker_thread.join(timeout=max(1.0, self._http_timeout_sec + 0.5))

        result, _meta = self._http_client.stop_motor_with_meta()
        if not bool(result.get("stopped", False)):
            self._safe_log(
                "warning",
                f"Failed to stop motor cleanly during shutdown: {result.get('error', 'unknown')}",
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[SpikeHwClientNode] = None
    try:
        node = SpikeHwClientNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.request_shutdown_cleanup()
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if rclpy.ok():
            rclpy.shutdown()
