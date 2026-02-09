# Pattern Schema (YAML)

Each pattern file uses this shape:

```yaml
name: "my_pattern"
version: 1
defaults:
  motor_port: "A"
  stop_action: "coast"  # coast|brake|hold
steps:
  - type: "run_for_time"
    velocity: 0.5
    duration_sec: 0.4
```

## Supported Step Types

- `run`: set velocity and keep running until another step changes it.
- `stop`: stop motor with optional `stop_action`.
- `sleep`: wait without changing motor state.
- `run_for_time`: run velocity for `duration_sec`, then stop.
- `run_for_degrees`: run velocity for `degrees`, then stop.
- `run_to_absolute_position`: move to absolute encoder position.
- `run_to_relative_position`: move relative encoder amount.
- `reset_relative_position`: reset relative encoder to zero.
- `set_duty_cycle`: best-effort duty cycle command.
- `beep`: trigger hub speaker beep.

## Canonical Units

- `velocity`: float in `[-1.0, 1.0]`.
- `degrees`: integer encoder degrees.
- `position_degrees`: integer absolute encoder position.
- `duration_sec`: float seconds.
- `direction`: optional `cw|ccw` (applies sign to velocity).
- `freq_hz`: integer frequency in `[50, 5000]` for `beep`.
- `duration_ms`: integer beep duration in `[10, 5000]`.
- `volume`: integer in `[0, 100]` (default `50`).

## Validation Rules

- Unknown top-level keys are rejected.
- Unknown step keys are rejected.
- Each step must include exactly the required keys for its type.
- Errors include the failing step index (`step[<index>]`) with reason.

## Required Keys Per Step Type

- `run`: `type`, `velocity`
- `stop`: `type`
- `sleep`: `type`, `duration_sec`
- `run_for_time`: `type`, `velocity`, `duration_sec`
- `run_for_degrees`: `type`, `velocity`, `degrees`
- `run_to_absolute_position`: `type`, `velocity`, `position_degrees`
- `run_to_relative_position`: `type`, `velocity`, `degrees`
- `reset_relative_position`: `type`
- `set_duty_cycle`: `type`, `velocity`
- `beep`: `type`, `freq_hz`, `duration_ms`

## Optional Keys

- `port` (`A`..`F`) step-level override.
- `stop_action` (`coast|brake|hold`) on stop-related steps.
- `direction` (`cw|ccw`) on velocity steps.
- `wait_sec` extra blocking wait after a step completes.
- `gap_sec` extra pause inserted after the step (after `wait_sec` if both are set).
- `comment` free-form string.
- `volume` on `beep` steps (default `50`).

## Beep Example

```yaml
- type: "beep"
  freq_hz: 440
  duration_ms: 120
  volume: 60
  gap_sec: 0.05
```
