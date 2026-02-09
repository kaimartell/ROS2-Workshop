# SPIKE ROS 2 Workshop Notebook

Use this as the live workshop handout (Notion-ready). Commands are copy/pasteable.

## 0) Outcome Checklist

By the end, you should be able to:
- [ ] Run host agent + ROS container.
- [ ] Launch the ROS graph once and keep it running.
- [ ] List/generate/play/stop patterns through ROS services.
- [ ] Use `pattern_menu` as a beginner-friendly interface.
- [ ] Read status from ROS topics.

## 1) Pre-Flight

- [ ] Docker Desktop running.
- [ ] Python 3.9+ installed.
- [ ] SPIKE hub connected with USB **data** cable.
- [ ] Motor connected to port A (or note your port).

Clone and install host deps:

```bash
git clone https://github.com/<your-org-or-user>/spike-ros-workshop.git
cd spike-ros-workshop

cd host_agent
python3 -m pip install -e ".[spike_usb]"
cd ..
```

What you should see:
- [ ] pip install succeeds.
- [ ] no import errors for `host_agent`.

## 2) Start Host Agent (Host OS)

```bash
./scripts/start_host_agent_usb.sh
```

What you should see:
- [ ] server listening on `http://127.0.0.1:8000`
- [ ] backend `spike_usb` (or `mock` if fallback)

Health check:

```bash
curl http://localhost:8000/health
```

What you should see:
- [ ] JSON with `ok: true`
- [ ] `backend` and `spike_connected` fields

### Rescue Path A: serial auto-detect fails

```bash
python3 -m host_agent --list
SPIKE_SERIAL=/dev/cu.usbmodemXXXX ./scripts/start_host_agent_usb.sh
```

### Rescue Path B: no serial devices appear

- [ ] confirm data cable
- [ ] close LEGO SPIKE app
- [ ] unplug/replug hub

Workshop fallback:

```bash
SPIKE_BACKEND=mock ./scripts/start_host_agent_usb.sh
```

## 3) Start ROS Container

```bash
./scripts/start_container.sh
```

Inside container:

```bash
ros2 --help >/dev/null && echo "ros2 ready"
```

What you should see:
- [ ] `ros2 ready`

### Rescue Path C: `ros2` not found

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

## 4) Launch Once (Idle-Ready)

Container terminal #1:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py
```

What you should see:
- [ ] `instrument_node` log showing service endpoints
- [ ] `spike_hw_client_node` running
- [ ] system remains idle until triggered

Container terminal #2 (sanity):

```bash
ros2 node list
ros2 topic list
ros2 service list | grep instrument
```

What you should see:
- [ ] nodes include `instrument_node`, `spike_hw_client_node`
- [ ] services include:
  - `/instrument/list_patterns`
  - `/instrument/generate_pattern`
  - `/instrument/play_pattern`
  - `/instrument/stop`

## 5) ROS-First Pattern Workflow

### 5.1 List patterns

```bash
ros2 service call /instrument/list_patterns std_srvs/srv/Trigger "{}"
```

What you should see:
- [ ] newline-separated names like `presets/pulse_4`, `user/<name>`

### 5.2 Generate a pattern from template

```bash
ros2 service call /instrument/generate_pattern spike_workshop_interfaces/srv/GeneratePattern "{template_name: 'pulse', output_name: 'my_pulse', output_dir: '/patterns/user', speed: 0.6, duration_sec: 0.3, gap_sec: 0.2, repeats: 4, bpm: 120.0, degrees: 180}"
```

What you should see:
- [ ] response `ok: true`
- [ ] `written_path` under `/patterns/user`

### 5.2b Generate a 4-bar score (single workflow)

```bash
ros2 service call /instrument/generate_pattern spike_workshop_interfaces/srv/GeneratePattern "{template_name: 'score_4bar', output_name: 'my_score', output_dir: '/patterns/user', speed: 0.5, duration_sec: 0.0, gap_sec: 0.0, repeats: 2, bpm: 120.0, degrees: 0, motor_score: 'F S F S | F S F S | B S B S | B S B S', melody_score: 'C4 D4 E4 F4 | G4 A4 B4 C5 | - - G4 - | C5 - - -', beep_volume: 60}"
```

What you should see:
- [ ] response `ok: true`
- [ ] generated file under `/patterns/user`

### 5.3 Play without relaunching

```bash
ros2 service call /instrument/play_pattern spike_workshop_interfaces/srv/PlayPattern "{pattern_name: 'my_pulse', pattern_path: ''}"
```

What you should see:
- [ ] response `accepted: true`
- [ ] motor pattern plays immediately

### 5.4 Observe while running

```bash
ros2 topic echo /status
ros2 topic echo /done
ros2 topic echo /spike/state
```

What you should see:
- [ ] `/status` transitions include step progress
- [ ] `/done` once when complete

### 5.5 Stop reliably

```bash
ros2 service call /instrument/stop std_srvs/srv/Trigger "{}"
```

What you should see:
- [ ] response success
- [ ] motor stops

## 6) Beginner Menu (No JSON Typing)

Inside container:

```bash
ros2 run spike_workshop_tools pattern_menu
```

What you should see:
- [ ] menu options: list / play / generate / stop
- [ ] template choices: `pulse`, `metronome_finite`, `bounce_encoder`, `clock_tick`
- [ ] printed equivalent `ros2 service call ...` commands

## 7) Pattern Preset Quick Reference

- `pulse_4.yaml`: finite pulses (`run_for_time` + gaps)
- `metronome_90.yaml`: finite tempo pulses
- `sweep_3s.yaml`: velocity sweep
- `dance_basic.yaml`: mixed timed + degree moves
- `clock_tick.yaml`: absolute encoder positioning
- `bounce_encoder.yaml`: relative encoder bounce
- `melody_scale.yaml`: C major speaker scale (beep steps)
- `melody_call_response.yaml`: beep phrase + motor response pattern
- `score_motor_basic.yaml`: 4-bar motor lane starter
- `score_melody_scale.yaml`: 4-bar melody lane starter
- `score_call_response.yaml`: 4-bar dual-lane call/response
- `score_back_and_forth_groove.yaml`: 4-bar dual-lane groove

Schema docs:
- `docs/pattern_schema.md`

## 8) Optional Shortcut Script (Fallback)

Service workflow is recommended. If needed, this shortcut still works:

```bash
./scripts/play_pattern.sh pulse_4
```

## 9) Troubleshooting Matrix

### Problem: `/spike/ping` fails

Check:

```bash
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
curl http://localhost:8000/health
```

Fix:
- [ ] ensure host agent is running
- [ ] launch with default `host_agent_url:=http://host.docker.internal:8000`

### Problem: `/instrument/play_pattern` rejected as busy

- [ ] call stop first:

```bash
ros2 service call /instrument/stop std_srvs/srv/Trigger "{}"
```

- [ ] relaunch with override if needed:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py allow_override:=true
```

### Problem: generated file not found

- [ ] list patterns again:

```bash
ros2 service call /instrument/list_patterns std_srvs/srv/Trigger "{}"
```

- [ ] confirm output dir in generate request (`/patterns/user` recommended)

### Problem: motor still moving after interruption

Manual stop:

```bash
curl -s -X POST http://localhost:8000/motor/stop \
  -H "Content-Type: application/json" \
  -d '{"port":"A"}'
```

## 10) Suggested Day Flow

- [ ] 00:00-00:15 setup + health checks
- [ ] 00:15-00:35 ROS nodes/topics/services overview
- [ ] 00:35-00:55 list/play preset patterns via service calls
- [ ] 00:55-01:20 generate custom patterns via service + `pattern_menu`
- [ ] 01:20-01:40 encoder/position labs (`clock_tick`, `bounce_encoder`)
- [ ] 01:40-02:00 debugging drills + share-out

## 11) Tiny Glossary

- **Node**: a running ROS process (`instrument_node`, `spike_hw_client_node`).
- **Topic**: streaming pub/sub channel (`/status`, `/spike/state`).
- **Service**: request/response RPC (`/instrument/play_pattern`).
- **Host agent**: host-side HTTP bridge to SPIKE hardware.
- **Pattern**: YAML sequence of motor steps saved on disk.
