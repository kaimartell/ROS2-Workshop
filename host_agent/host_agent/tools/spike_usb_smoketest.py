import argparse
import sys

from host_agent.backends.spike_usb_backend import SpikeUsbBackend


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Smoke test for SPIKE USB backend (non-blocking motor.run then explicit stop)"
    )
    parser.add_argument("--serial-port", default="auto", help="Serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud")
    parser.add_argument("--motor-port", default="A", help="Motor port letter (A-F)")
    parser.add_argument("--speed", type=float, default=0.4, help="Motor speed in [-1.0, 1.0]")
    parser.add_argument("--duration", type=float, default=0.8, help="Run duration in seconds")
    return parser.parse_args()


def _clip(text: str, limit: int = 600) -> str:
    normalized = (text or "").strip()
    if len(normalized) <= limit:
        return normalized
    return f"{normalized[:limit]}...(truncated)"


def _print_diag(payload: dict) -> None:
    stdout = _clip(str(payload.get("stdout", "")))
    stderr = _clip(str(payload.get("stderr", "")))
    if stdout:
        print(f"stdout: {stdout}")
    if stderr:
        print(f"stderr: {stderr}")


def main() -> None:
    args = parse_args()

    backend = SpikeUsbBackend(
        serial_port=args.serial_port,
        baud=args.baud,
        motor_port=args.motor_port,
    )

    health = backend.health()
    print(f"health: {health}")
    if not bool(health.get("spike_connected", False)):
        print("FAIL: spike_connected is false")
        if "detail" in health:
            print(f"detail: {health['detail']}")
        sys.exit(2)

    run_result = backend.run_with_diagnostics(speed=args.speed, duration=args.duration)
    print(f"run: {run_result}")
    if not bool(run_result.get("accepted", False)):
        print("FAIL: run command was not accepted")
        _print_diag(run_result)
        sys.exit(3)

    stop_result = backend.stop_with_diagnostics()
    if not bool(stop_result.get("stopped", False)):
        print("WARN: explicit stop command failed after run")
        print(f"stop: {stop_result}")
        _print_diag(stop_result)

    print("PASS: spike_usb non-blocking run + explicit stop completed")


if __name__ == "__main__":
    main()
