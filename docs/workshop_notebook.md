# SPIKE ROS 2 Workshop Notebook

Use this as the live handout. The workshop path is ROS-first:
1) launch once
2) ping hardware
3) generate a 4-bar score
4) play
5) stop

## 0) Outcome Checklist

- [ ] Host agent is running (`/health` returns `ok: true`)
- [ ] ROS graph is launched and idle-ready
- [ ] `/spike/ping` succeeds
- [ ] `/instrument/generate_score` creates a YAML in `/patterns/user`
- [ ] `/instrument/play_pattern` plays the score
- [ ] `/instrument/stop` stops motion

## 1) Pre-Flight (Host)

- [ ] Docker Desktop is running
- [ ] Python 3.10+ installed
- [ ] SPIKE hub connected with USB data cable
- [ ] Motor connected to port A

Install host agent deps:

```bash
cd host_agent
python3 -m pip install -e ".[spike_usb]"
cd ..
```

What you should see:
- [ ] pip install completes without errors

## 2) Start Host Agent

```bash
./scripts/start_host_agent_usb.sh
```

Verify:

```bash
curl http://localhost:8000/health
```

What you should see:
- [ ] JSON response with `ok: true`
- [ ] `backend` and `spike_connected` fields present

Rescue path (serial selection):

```bash
./scripts/start_host_agent_usb.sh --list
SPIKE_SERIAL=/dev/cu.usbmodemXXXX ./scripts/start_host_agent_usb.sh
```

## 3) Start Container + Launch ROS

Terminal A:

```bash
./scripts/start_container.sh
```

Inside container terminal:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py
```

Terminal B (new shell):

```bash
docker exec -it spike-workshop-participant bash
```

What you should see:
- [ ] `instrument_node` and `spike_hw_client_node` logs
- [ ] system stays idle until commands arrive

## 4) Connectivity Checks

Run in terminal B:

```bash
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
ros2 service call /instrument/list_patterns std_srvs/srv/Trigger "{}"
```

What you should see:
- [ ] ping `success: true`
- [ ] patterns list includes preset names

## 5) Generate a 4-Bar Score

Score format:
- motor lane: `F`, `B`, `S`
- melody lane: note (`C4`, `A#4`) or Hz (`440`) or rest (`-`)
- 4 bars of 4 beats: `bar1 | bar2 | bar3 | bar4`

Generate:

```bash
ros2 service call /instrument/generate_score spike_workshop_interfaces/srv/GenerateScore "{name: 'my_score', bpm: 120.0, repeats: 2, speed: 0.5, motor: 'F S F S | F S F S | B S B S | B S B S', melody: 'C4 D4 E4 F4 | G4 A4 B4 C5 | - - G4 - | C5 - - -', volume: 60}"
```

What you should see:
- [ ] `ok: true`
- [ ] output path in `/patterns/user/my_score.yaml`

## 6) Play + Stop

Play:

```bash
ros2 service call /instrument/play_pattern spike_workshop_interfaces/srv/PlayPattern "{pattern_name: 'my_score', pattern_path: ''}"
```

Stop:

```bash
ros2 service call /instrument/stop std_srvs/srv/Trigger "{}"
```

What you should see:
- [ ] play returns `accepted: true`
- [ ] status changes running -> idle
- [ ] stop returns `success: true`

## 7) Debugging Commands

```bash
ros2 node list
ros2 topic list
ros2 service list | grep instrument
ros2 topic echo /status
ros2 topic echo /done
ros2 topic echo /spike/state
```

## 8) Troubleshooting Quick Paths

`spike_connected: false`:
- close LEGO SPIKE app
- re-run `./scripts/start_host_agent_usb.sh --list`
- pass explicit `SPIKE_SERIAL`

No motor motion:
- check `ros2 service call /spike/ping ...`
- check host agent health
- confirm motor is on configured port

No beep:
- firmware must expose `hub.sound.beep`
- run melody score and inspect host agent logs for sound error

Emergency stop:

```bash
curl -s -X POST http://localhost:8000/motor/stop \
  -H "Content-Type: application/json" \
  -d '{"port":"A"}'
```
