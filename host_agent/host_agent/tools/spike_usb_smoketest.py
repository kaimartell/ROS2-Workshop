import argparse
import sys

from host_agent.backends.spike_usb_backend import SpikeUsbBackend


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Smoke test for SPIKE USB backend (motor + optional beep)"
    )
    parser.add_argument("--serial-port", default="auto", help="Serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud")
    parser.add_argument("--motor-port", default="A", help="Motor port letter (A-F)")
    parser.add_argument("--speed", type=float, default=0.4, help="Motor speed in [-1.0, 1.0]")
    parser.add_argument("--duration", type=float, default=0.8, help="Run duration in seconds")
    parser.add_argument(
        "--debug-repl-snippets",
        action="store_true",
        help="Include snippet text in backend errors",
    )
    parser.add_argument(
        "--beep",
        action="store_true",
        help="Also run a speaker beep test via hub.sound.beep",
    )
    parser.add_argument("--beep-freq", type=int, default=440, help="Beep frequency in Hz")
    parser.add_argument("--beep-duration-ms", type=int, default=140, help="Beep duration in ms")
    parser.add_argument("--beep-volume", type=int, default=60, help="Beep volume 0..100")
    return parser.parse_args()


def _clip(text: str, limit: int = 600) -> str:
    normalized = (text or "").strip()
    if len(normalized) <= limit:
        return normalized
    return f"{normalized[:limit]}...(truncated)"


def _print_diag(payload: dict) -> None:
    stdout = _clip(str(payload.get("stdout", "")))
    stderr = _clip(str(payload.get("stderr", "")))
    snippet = _clip(str(payload.get("snippet", "")), limit=1400)
    if stdout:
        print(f"stdout: {stdout}")
    if stderr:
        print(f"stderr: {stderr}")
    if snippet:
        print(f"snippet: {snippet}")


def main() -> None:
    args = parse_args()

    backend = SpikeUsbBackend(
        serial_port=args.serial_port,
        baud=args.baud,
        motor_port=args.motor_port,
        debug_repl_snippets=args.debug_repl_snippets,
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

    if args.beep:
        beep_result = backend.sound_beep(
            freq_hz=args.beep_freq,
            duration_ms=args.beep_duration_ms,
            volume=args.beep_volume,
        )
        print(f"beep: {beep_result}")
        if not bool(beep_result.get("accepted", False)):
            print("FAIL: beep command was not accepted")
            _print_diag(beep_result)
            sys.exit(4)

    print("PASS: spike_usb non-blocking run + explicit stop completed")


if __name__ == "__main__":
    main()
