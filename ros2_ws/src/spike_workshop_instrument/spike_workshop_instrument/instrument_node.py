import json
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from spike_workshop_instrument.behaviors import build_steps


class InstrumentNode(Node):
    def __init__(self) -> None:
        super().__init__("instrument_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("participant_id", 1),
                ("name", "instrument"),
                ("mode", "pulse"),
                ("speed", 0.6),
                ("duration", 1.0),
                ("repeats", 4),
                ("bpm", 120.0),
                ("amplitude", 0.6),
                ("sweep_steps", 8),
                ("seed", 0),
                ("sequence_file", ""),
                ("cmd_topic", "/spike/cmd"),
                ("done_topic", "/done"),
                ("status_topic", "/status"),
            ],
        )

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        done_topic = str(self.get_parameter("done_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self._cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self._done_pub = self.create_publisher(Empty, done_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._actuate_sub = self.create_subscription(Empty, "/actuate", self._on_actuate, 10)
        self._spike_state_sub = self.create_subscription(
            String, "/spike/state", self._on_spike_state, 10
        )

        self._status_timer = self.create_timer(0.5, self._publish_status)

        self._lock = threading.Lock()
        self._running = False
        self._last_action = "idle"
        self._worker: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._host_connected = False

        self.get_logger().info(
            "instrument_node ready. Trigger behavior with: ros2 topic pub /actuate std_msgs/msg/Empty '{}' -1"
        )

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

    def _on_actuate(self, _msg: Empty) -> None:
        with self._lock:
            if self._running:
                self._last_action = "busy"
                busy = True
            else:
                self._running = True
                self._last_action = "starting"
                busy = False

        if busy:
            self._safe_log("info", "Received /actuate while running; ignoring trigger.")
            self._publish_status()
            return

        self._worker = threading.Thread(target=self._run_behavior, daemon=True)
        self._worker.start()
        self._publish_status()

    def _run_behavior(self) -> None:
        params = self._read_behavior_params()

        try:
            steps = build_steps(**params)
        except Exception as exc:  # noqa: BLE001
            self._safe_log("error", f"Failed to construct behavior steps: {exc}")
            with self._lock:
                self._running = False
                self._last_action = f"error: {exc}"
            self._publish_status()
            return

        mode = str(params["mode"])
        self._safe_log("info", f"Executing mode={mode} with {len(steps)} steps")

        for index, step in enumerate(steps, start=1):
            if self._stop_event.is_set():
                break

            speed = float(step.get("speed", 0.0))
            step_duration = max(0.0, float(step.get("duration", 0.0)))

            if not self._publish_motor_command(speed=speed, duration=step_duration):
                break

            with self._lock:
                self._last_action = (
                    f"{mode} step {index}/{len(steps)} speed={speed:.2f} duration={step_duration:.2f}s"
                )
            self._publish_status()

            if not self._sleep_with_cancel(step_duration):
                break

        if not self._stop_event.is_set() and self._context_ok():
            try:
                self._done_pub.publish(Empty())
                self._safe_log("info", "Behavior complete; published /done")
            except Exception:  # noqa: BLE001
                pass

        with self._lock:
            self._running = False
            self._last_action = "idle"

        self._publish_motor_command(speed=0.0, duration=0.0, log_command=False)
        self._publish_status()

    def _on_spike_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = str(payload.get("state", ""))
        with self._lock:
            self._host_connected = state not in {"", "unreachable"}

    def _sleep_with_cancel(self, total_seconds: float) -> bool:
        remaining = max(0.0, float(total_seconds))
        while remaining > 0.0:
            if self._stop_event.is_set():
                return False
            chunk = min(0.05, remaining)
            time.sleep(chunk)
            remaining -= chunk
        return True

    def _read_behavior_params(self) -> Dict[str, Any]:
        return {
            "mode": str(self.get_parameter("mode").value),
            "speed": float(self.get_parameter("speed").value),
            "duration": float(self.get_parameter("duration").value),
            "repeats": int(self.get_parameter("repeats").value),
            "bpm": float(self.get_parameter("bpm").value),
            "amplitude": float(self.get_parameter("amplitude").value),
            "sweep_steps": int(self.get_parameter("sweep_steps").value),
            "seed": int(self.get_parameter("seed").value),
            "sequence_file": str(self.get_parameter("sequence_file").value),
        }

    def _publish_motor_command(
        self, *, speed: float, duration: float, log_command: bool = True
    ) -> bool:
        if not self._context_ok():
            return False
        payload = {"speed": float(speed), "duration": max(0.0, float(duration))}
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        try:
            self._cmd_pub.publish(msg)
        except Exception:  # noqa: BLE001
            return False
        if log_command:
            self._safe_log("info", f"Published /spike/cmd {msg.data}")
        return True

    def _build_status_text(self) -> str:
        with self._lock:
            state = "running" if self._running else "idle"
            last_action = self._last_action
            host_connected = self._host_connected

        return (
            f"state={state};last_action={last_action};"
            f"participant_id={int(self.get_parameter('participant_id').value)};"
            f"name={str(self.get_parameter('name').value)};"
            f"mode={str(self.get_parameter('mode').value)};"
            f"host_connected={str(host_connected).lower()}"
        )

    def _publish_status(self) -> None:
        if not self._context_ok():
            return
        msg = String()
        msg.data = self._build_status_text()
        try:
            self._status_pub.publish(msg)
        except Exception:  # noqa: BLE001
            pass

    def request_shutdown_cleanup(self) -> None:
        self._stop_event.set()

        if self._status_timer is not None:
            self._status_timer.cancel()

        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=2.0)

        if self._context_ok():
            self._publish_motor_command(speed=0.0, duration=0.0, log_command=False)
        else:
            # ROS context already shut down; skip publish to avoid invalid-context errors.
            pass


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[InstrumentNode] = None
    try:
        node = InstrumentNode()
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
