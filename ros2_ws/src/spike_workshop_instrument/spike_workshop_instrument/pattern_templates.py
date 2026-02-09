from __future__ import annotations

import datetime as dt
from pathlib import Path
from typing import Any, Dict, List

import yaml
from spike_workshop_instrument.score_utils import (
    MOTOR_PRESETS_REST,
    MELODY_PRESETS_REST,
    parse_melody_score,
    parse_motor_score,
    resolve_score_preset,
)


TEMPLATES = ("pulse", "metronome_finite", "bounce_encoder", "clock_tick", "score_4bar")
SCORE_DEFAULT_BEEP_DURATION_MS = 100


def _timestamped_name(prefix: str) -> str:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{stamp}"


def _normalized_name(name: str, fallback_prefix: str) -> str:
    raw = str(name or "").strip()
    if not raw:
        raw = _timestamped_name(fallback_prefix)
    if raw.lower().endswith(".yaml"):
        raw = raw[:-5]
    safe = "".join(ch if ch.isalnum() or ch in {"_", "-", "."} else "_" for ch in raw).strip("_")
    return safe or _timestamped_name(fallback_prefix)


def _base_pattern(name: str, motor_port: str, stop_action: str, steps: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "name": name,
        "version": 1,
        "defaults": {
            "motor_port": motor_port,
            "stop_action": stop_action,
        },
        "steps": steps,
    }


def _pulse_steps(speed: float, duration_sec: float, gap_sec: float, repeats: int) -> List[Dict[str, Any]]:
    steps: List[Dict[str, Any]] = []
    count = max(1, int(repeats))
    run_duration = max(0.02, float(duration_sec))
    gap = max(0.0, float(gap_sec))
    for idx in range(count):
        steps.append(
            {
                "type": "run_for_time",
                "velocity": speed,
                "duration_sec": run_duration,
            }
        )
        if idx < count - 1 and gap > 0.0:
            steps.append({"type": "sleep", "duration_sec": gap})
    return steps


def _metronome_steps(
    speed: float,
    duration_sec: float,
    gap_sec: float,
    repeats: int,
    bpm: float,
) -> List[Dict[str, Any]]:
    count = max(1, int(repeats))
    beat_period = 60.0 / max(1.0, float(bpm))
    on_time = float(duration_sec)
    if on_time <= 0.0:
        on_time = min(0.1, beat_period * 0.3)
    on_time = max(0.02, min(on_time, beat_period))
    off_time = max(0.0, beat_period - on_time)
    extra_gap = max(0.0, float(gap_sec))

    steps: List[Dict[str, Any]] = []
    for idx in range(count):
        steps.append({"type": "run_for_time", "velocity": speed, "duration_sec": on_time})
        remaining = off_time
        if idx < count - 1:
            remaining += extra_gap
        if remaining > 0.0:
            steps.append({"type": "sleep", "duration_sec": remaining})
    return steps


def _bounce_steps(speed: float, gap_sec: float, repeats: int, degrees: int) -> List[Dict[str, Any]]:
    count = max(1, int(repeats))
    magnitude = max(10, abs(int(degrees)))
    velocity = max(0.05, min(1.0, abs(float(speed))))
    gap = max(0.0, float(gap_sec))

    steps: List[Dict[str, Any]] = [{"type": "reset_relative_position"}]
    for idx in range(count):
        delta = magnitude if idx % 2 == 0 else -magnitude
        steps.append(
            {
                "type": "run_to_relative_position",
                "velocity": velocity,
                "degrees": delta,
                "stop_action": "hold",
            }
        )
        if idx < count - 1 and gap > 0.0:
            steps.append({"type": "sleep", "duration_sec": gap})
    return steps


def _clock_steps(speed: float, gap_sec: float, repeats: int, degrees: int) -> List[Dict[str, Any]]:
    count = max(1, int(repeats))
    step_angle = abs(int(degrees))
    if step_angle == 0:
        step_angle = 90
    velocity = max(0.05, min(1.0, abs(float(speed))))
    gap = max(0.0, float(gap_sec))

    steps: List[Dict[str, Any]] = [{"type": "reset_relative_position"}]
    for idx in range(count):
        target = (idx * step_angle) % 360
        steps.append(
            {
                "type": "run_to_absolute_position",
                "velocity": velocity,
                "position_degrees": int(target),
                "stop_action": "hold",
            }
        )
        if idx < count - 1 and gap > 0.0:
            steps.append({"type": "sleep", "duration_sec": gap})
    return steps


def _score_steps(
    *,
    bpm: float,
    repeats: int,
    speed: float,
    gap_sec: float,
    motor_port: str,
    beep_volume: int,
    motor_score: str,
    melody_score: str,
) -> List[Dict[str, Any]]:
    beat_sec = 60.0 / max(1.0, float(bpm))
    beat_gap = max(0.0, float(gap_sec))
    active_sec = max(0.02, beat_sec - beat_gap)
    beep_duration_sec = float(SCORE_DEFAULT_BEEP_DURATION_MS) / 1000.0
    speed_abs = max(0.0, min(1.0, abs(float(speed))))
    volume = max(0, min(100, int(beep_volume)))

    motor_lane = parse_motor_score(motor_score)
    melody_lane = parse_melody_score(melody_score)

    steps: List[Dict[str, Any]] = []
    loop_count = max(1, int(repeats))

    for _loop in range(loop_count):
        for beat_index in range(16):
            motor_token = motor_lane[beat_index]
            melody_freq = melody_lane[beat_index]

            motor_running = False
            if motor_token == "F" and speed_abs > 0.0:
                steps.append(
                    {
                        "type": "run",
                        "velocity": speed_abs,
                        "port": motor_port,
                    }
                )
                motor_running = True
            elif motor_token == "B" and speed_abs > 0.0:
                steps.append(
                    {
                        "type": "run",
                        "velocity": -speed_abs,
                        "port": motor_port,
                    }
                )
                motor_running = True
            else:
                steps.append(
                    {
                        "type": "stop",
                        "port": motor_port,
                    }
                )

            elapsed_sec = 0.0
            if melody_freq is not None:
                steps.append(
                    {
                        "type": "beep",
                        "freq_hz": int(melody_freq),
                        "duration_ms": SCORE_DEFAULT_BEEP_DURATION_MS,
                        "volume": volume,
                    }
                )
                elapsed_sec += beep_duration_sec

            remaining_active = max(0.0, active_sec - elapsed_sec)
            if remaining_active > 0.0:
                steps.append({"type": "sleep", "duration_sec": round(remaining_active, 4)})

            if motor_running:
                steps.append({"type": "stop", "port": motor_port})

            if beat_gap > 0.0:
                steps.append({"type": "sleep", "duration_sec": round(beat_gap, 4)})

    return steps


def build_template_pattern(
    *,
    template_name: str,
    output_name: str,
    speed: float,
    duration_sec: float,
    gap_sec: float,
    repeats: int,
    bpm: float,
    degrees: int,
    motor_score: str = "",
    melody_score: str = "",
    beep_volume: int = 60,
    motor_port: str = "A",
) -> Dict[str, Any]:
    template = str(template_name or "").strip().lower()
    if template not in TEMPLATES:
        raise ValueError(
            f"unsupported template '{template_name}', expected one of {list(TEMPLATES)}"
        )

    speed_clamped = max(-1.0, min(1.0, float(speed)))
    name = _normalized_name(output_name, fallback_prefix=template)
    port = str(motor_port or "A").strip().upper() or "A"

    if template == "pulse":
        return _base_pattern(
            name=name,
            motor_port=port,
            stop_action="coast",
            steps=_pulse_steps(
                speed=speed_clamped,
                duration_sec=duration_sec,
                gap_sec=gap_sec,
                repeats=repeats,
            ),
        )

    if template == "metronome_finite":
        return _base_pattern(
            name=name,
            motor_port=port,
            stop_action="coast",
            steps=_metronome_steps(
                speed=speed_clamped,
                duration_sec=duration_sec,
                gap_sec=gap_sec,
                repeats=repeats,
                bpm=bpm,
            ),
        )

    if template == "bounce_encoder":
        return _base_pattern(
            name=name,
            motor_port=port,
            stop_action="hold",
            steps=_bounce_steps(
                speed=speed_clamped,
                gap_sec=gap_sec,
                repeats=repeats,
                degrees=degrees,
            ),
        )

    if template == "score_4bar":
        motor_lane_raw = str(motor_score or "").strip()
        melody_lane_raw = str(melody_score or "").strip()

        preset_choice = resolve_score_preset(motor_lane_raw.lower())
        if preset_choice is not None and not melody_lane_raw:
            motor_lane_raw, melody_lane_raw = preset_choice

        if not motor_lane_raw and not melody_lane_raw:
            motor_lane_raw = MOTOR_PRESETS_REST
            melody_lane_raw = MELODY_PRESETS_REST

        steps = _score_steps(
            bpm=bpm,
            repeats=repeats,
            speed=speed_clamped,
            gap_sec=gap_sec,
            motor_port=port,
            beep_volume=beep_volume,
            motor_score=motor_lane_raw,
            melody_score=melody_lane_raw,
        )
        return _base_pattern(
            name=name,
            motor_port=port,
            stop_action="coast",
            steps=steps,
        )

    return _base_pattern(
        name=name,
        motor_port=port,
        stop_action="hold",
        steps=_clock_steps(
            speed=speed_clamped,
            gap_sec=gap_sec,
            repeats=repeats,
            degrees=degrees,
        ),
    )


def ensure_yaml_output_path(*, output_dir: str, output_name: str) -> Path:
    directory = Path(output_dir or "/patterns/user")
    filename = _normalized_name(output_name, fallback_prefix="pattern")
    if not filename.endswith(".yaml"):
        filename = f"{filename}.yaml"
    return directory / filename


def write_pattern_yaml(path: Path, pattern: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rendered = yaml.safe_dump(pattern, sort_keys=False)
    path.write_text(rendered, encoding="utf-8")
