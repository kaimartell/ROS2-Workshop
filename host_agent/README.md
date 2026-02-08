# Host Agent

This process runs on the participant's **host OS** (macOS), outside Docker.
The ROS container communicates with it via `http://host.docker.internal:8000`.

## Quick Start

Run mock backend (recommended default for workshop reliability):

```bash
python3 -m host_agent --port 8000 --backend mock
```

Run USB backend (SPIKE Prime over USB serial MicroPython REPL):

```bash
python3 -m host_agent --port 8000 --backend spike_usb --serial-port auto --motor-port A
```

Run BLE backend (optional):

```bash
python3 -m host_agent --port 8000 --backend spike_ble
```

List helper (serial + BLE):

```bash
python3 -m host_agent --list
```

## Dependencies

Install optional USB dependency:

```bash
python3 -m pip install ".[spike_usb]"
```

Install optional BLE dependency:

```bash
python3 -m pip install ".[spike_ble]"
```

## USB Backend Notes (macOS)

`spike_usb` targets the official LEGO SPIKE Prime firmware API style using:
- `from hub import port`
- `import motor`
- `motor.run(...)` / `motor.stop(...)`

Manual REPL test snippet:

```python
import motor
from hub import port
motor.run(port.A, 1000)
```

`spike_usb` uses non-blocking run semantics:
- `POST /motor/run` starts motion immediately (`motor.run(...)`) and returns quickly.
- Timing is controlled by ROS sending a later stop command.
- `POST /motor/stop` performs `motor.stop(port.X)` with fallbacks (`motor.run(..., 0)`, then `motor.set_duty_cycle(..., 0)`).

It tries to discover serial devices such as:
- `/dev/cu.usbmodem*`
- `/dev/cu.usbserial*`
- `/dev/cu.SLAB_USBtoUART*`

If `--serial-port auto` finds exactly one candidate, it uses it.
If it finds multiple, pass the explicit port:

```bash
python3 -m host_agent --backend spike_usb --serial-port /dev/cu.usbmodemXXXX --motor-port A
```

You can also set baud (default `115200`):

```bash
python3 -m host_agent --backend spike_usb --serial-port /dev/cu.usbmodemXXXX --baud 115200 --motor-port A
```

Quick manual macOS check:

```bash
ls /dev/cu.*
```

Compare this list with hub unplugged vs plugged.

## USB Smoke Test

Direct backend test without HTTP server:

```bash
python3 -m host_agent.tools.spike_usb_smoketest --serial-port auto --motor-port A
```

## Troubleshooting

- If no serial device appears, verify USB cable is a **data cable** (LEGO official cable can matter).
- Ensure LEGO SPIKE app is closed; it can hold the serial port.
- If only BLE devices appear but no serial device, check hub mode/firmware and cable.
- If run fails with “Not running LEGO SPIKE firmware / knowledge base API.”, the hub is not exposing the expected LEGO API REPL.
- Backend errors should not crash the server; `/health` reports `spike_connected: false` when unavailable.

## API

- `GET /health` -> `{ "ok": true, "backend": "mock|spike_ble|spike_usb", "spike_connected": bool }`
- `POST /motor/run` with `{ "speed": float, "duration": float }` -> `{ "accepted": true|false, "note": "nonblocking; duration handled by ROS stop commands", "error": "..." }`
- `POST /motor/stop` -> `{ "stopped": true|false, "error": "..." }`
- `GET /state` -> `{ "state": "idle|running", "last_speed": 0.0, "timestamp": ... }`
