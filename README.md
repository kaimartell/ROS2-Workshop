# spike-ros-workshop

Build a physical instrument using ROS 2 and LEGO SPIKE Prime.

`spike-ros-workshop` is a beginner-friendly workshop project where participants use ROS commands instead of writing code. You run one host-side service, start one Dockerized ROS environment, launch one ROS app, then trigger the instrument with CLI commands.

The focus is learning core ROS ideas in a hands-on way: nodes, topics, parameters, services, and observability. The provided defaults work in `mock` mode even without hardware, and switch to real SPIKE hardware when USB is available.

## What this workshop is

This workshop gives each participant their own local ROS 2 graph. The instrument behavior is already implemented and configurable through launch arguments like `mode`, `speed`, `duration`, `repeats`, and `bpm`.

Participants inspect and control the system with `ros2 topic`, `ros2 service`, and `ros2 launch`. No coding is required during the workshop.

## System architecture

1. Host computer (macOS) connects to LEGO SPIKE Prime over USB.
2. `host_agent` runs on macOS and exposes a simple HTTP API (`/health`, `/motor/run`, `/motor/stop`, `/state`).
3. Dockerized ROS 2 container talks to `host_agent` via `http://host.docker.internal:8000`.
4. `spike_hw_client_node` bridges ROS topics/services to host HTTP.
5. `instrument_node` publishes motor commands and status based on launch parameters.

## Prerequisites

- macOS (Apple Silicon or Intel)
- Docker Desktop
- Python 3.9+
- LEGO SPIKE Prime hub + motor + USB data cable

## Quick start (10-minute path)

1. Clone the repository:

```bash
git clone https://github.com/kaimartell/spike-ros-workshop.git
cd spike-ros-workshop
```

2. Install host agent dependencies (USB backend):

```bash
cd host_agent
python3 -m pip install -e ".[spike_usb]"
cd ..
```

3. Start host agent (USB mode):

```bash
cd host_agent
python3 -m host_agent --port 8000 --backend spike_usb --serial-port auto --motor-port A
```

If you are not using hardware yet, run mock instead:

```bash
python3 -m host_agent --port 8000 --backend mock
```

4. In a new terminal, run the ROS container:

```bash
cd spike-ros-workshop
./scripts/run.sh
```

5. Inside the container, launch the instrument:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py mode:=pulse speed:=0.6 duration:=1.0 repeats:=4
```

6. Trigger the motor/instrument behavior:

```bash
ros2 topic pub /actuate std_msgs/msg/Empty "{}" -1
```

## Key ROS commands

Check connectivity to host agent:

```bash
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
```

Observe state and status:

```bash
ros2 topic echo /status
ros2 topic echo /spike/state
```

Trigger one actuation:

```bash
ros2 topic pub /actuate std_msgs/msg/Empty "{}" -1
```

Use launch parameters to customize behavior:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py \
  mode:=metronome bpm:=120 repeats:=16 speed:=0.5 duration:=0.3
```

Common launch parameters:
- `mode` (`pulse|sweep|metronome|random_wiggle|sequence`)
- `speed`, `duration`, `repeats`, `bpm`, `amplitude`
- `host_agent_url` (default `http://host.docker.internal:8000`)

## Troubleshooting

Serial port selection:
- List candidates: `python3 -m host_agent --list`
- If multiple serial ports exist, pick one explicitly:
  `python3 -m host_agent --backend spike_usb --serial-port /dev/cu.usbmodemXXXX --motor-port A`

SPIKE app conflicts:
- Close the LEGO SPIKE app before running `host_agent`; it can lock the serial port.

Motor not moving:
- Check host agent health: `curl http://localhost:8000/health`
- Check ROS connectivity: `ros2 service call /spike/ping std_srvs/srv/Trigger "{}"`
- Try mock mode to isolate ROS path from hardware:
  `python3 -m host_agent --port 8000 --backend mock`

## What’s next

- Multi-participant orchestration and shared timing
- Custom sequence editing for performances
- Extending instrument behaviors and hardware backends
