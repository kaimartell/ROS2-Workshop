#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from spike_workshop_interfaces.srv import GeneratePattern, PlayPattern
from std_srvs.srv import Trigger


SCORE_REST_MOTOR = "S S S S | S S S S | S S S S | S S S S"
SCORE_REST_MELODY = "- - - - | - - - - | - - - - | - - - -"

SCORE_PRESETS = {
    "motor_basic": {
        "motor_score": "F S F S | F S F S | B S B S | B S B S",
        "melody_score": SCORE_REST_MELODY,
        "description": "Motor-only starter groove.",
    },
    "melody_scale": {
        "motor_score": SCORE_REST_MOTOR,
        "melody_score": "C4 D4 E4 F4 | G4 A4 B4 C5 | C5 B4 A4 G4 | F4 E4 D4 C4",
        "description": "Melody-only C-major up/down.",
    },
    "call_response": {
        "motor_score": "F F S S | B B S S | F F S S | B B S S",
        "melody_score": "C4 E4 - - | G4 E4 - - | C5 B4 - - | A4 G4 - -",
        "description": "Alternating phrases for motor + melody.",
    },
    "back_and_forth_groove": {
        "motor_score": "F B F B | F B F B | B F B F | B F B F",
        "melody_score": "A4 - A4 - | C5 - C5 - | G4 - G4 - | E4 - E4 -",
        "description": "Busy motion with simple melodic hits.",
    },
}

MELODY_OVERLAYS = {
    "scale": SCORE_PRESETS["melody_scale"]["melody_score"],
    "call_response": SCORE_PRESETS["call_response"]["melody_score"],
    "groove": SCORE_PRESETS["back_and_forth_groove"]["melody_score"],
}


def _ask_text(prompt: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    raw = input(f"{prompt}{suffix}: ").strip()
    if raw:
        return raw
    return default


def _ask_float(prompt: str, default: float) -> float:
    while True:
        raw = _ask_text(prompt, str(default))
        try:
            return float(raw)
        except ValueError:
            print("Please enter a numeric value.")


def _ask_int(prompt: str, default: int) -> int:
    while True:
        raw = _ask_text(prompt, str(default))
        try:
            return int(raw)
        except ValueError:
            print("Please enter an integer value.")


def _ask_yes_no(prompt: str, default_yes: bool = True) -> bool:
    default = "y" if default_yes else "n"
    while True:
        raw = _ask_text(f"{prompt} (y/n)", default).strip().lower()
        if raw in {"y", "yes"}:
            return True
        if raw in {"n", "no"}:
            return False
        print("Please answer y or n.")


def _ask_choice(prompt: str, values: tuple[str, ...], default: str) -> str:
    choices = ", ".join(values)
    lowered = {value.lower(): value for value in values}
    while True:
        raw = _ask_text(f"{prompt} ({choices})", default).strip().lower()
        if raw in lowered:
            return lowered[raw]
        print(f"Please choose one of: {choices}")


class PatternMenuClient:
    def __init__(self) -> None:
        self._node = Node("pattern_menu")
        self._list_client = self._node.create_client(Trigger, "/instrument/list_patterns")
        self._stop_client = self._node.create_client(Trigger, "/instrument/stop")
        self._play_client = self._node.create_client(PlayPattern, "/instrument/play_pattern")
        self._generate_client = self._node.create_client(
            GeneratePattern, "/instrument/generate_pattern"
        )

    def close(self) -> None:
        self._node.destroy_node()

    def _call_service(self, client, request, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None, "service unavailable"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            return None, "service call timeout"
        if future.exception() is not None:
            return None, str(future.exception())
        return future.result(), ""

    def list_patterns(self) -> Tuple[bool, str]:
        response, error = self._call_service(self._list_client, Trigger.Request(), 3.0)
        if response is None:
            return False, error
        return bool(response.success), str(response.message)

    def stop(self) -> Tuple[bool, str]:
        response, error = self._call_service(self._stop_client, Trigger.Request(), 3.0)
        if response is None:
            return False, error
        return bool(response.success), str(response.message)

    def play(self, *, pattern_name: str, pattern_path: str) -> Tuple[bool, str]:
        request = PlayPattern.Request()
        request.pattern_name = str(pattern_name)
        request.pattern_path = str(pattern_path)
        response, error = self._call_service(self._play_client, request, 5.0)
        if response is None:
            return False, error
        return bool(response.accepted), str(response.message)

    def generate_score(
        self,
        *,
        output_name: str,
        output_dir: str,
        speed: float,
        gap_sec: float,
        repeats: int,
        bpm: float,
        motor_score: str,
        melody_score: str,
        beep_volume: int,
    ) -> Tuple[bool, str, str]:
        request = GeneratePattern.Request()
        request.template_name = "score_4bar"
        request.output_name = str(output_name)
        request.output_dir = str(output_dir)
        request.speed = float(speed)
        request.duration_sec = 0.0
        request.gap_sec = float(gap_sec)
        request.repeats = int(repeats)
        request.bpm = float(bpm)
        request.degrees = 0
        request.motor_score = str(motor_score)
        request.melody_score = str(melody_score)
        request.beep_volume = int(beep_volume)

        response, error = self._call_service(self._generate_client, request, 10.0)
        if response is None:
            return False, error, ""
        return bool(response.ok), str(response.message), str(response.written_path)


def _fmt_service_call_play(pattern_name: str, pattern_path: str) -> str:
    return (
        "ros2 service call /instrument/play_pattern "
        "spike_workshop_interfaces/srv/PlayPattern "
        f"\"{{pattern_name: '{pattern_name}', pattern_path: '{pattern_path}'}}\""
    )


def _fmt_service_call_generate_score(
    *,
    output_name: str,
    output_dir: str,
    speed: float,
    gap_sec: float,
    repeats: int,
    bpm: float,
    motor_score: str,
    melody_score: str,
    beep_volume: int,
) -> str:
    return (
        "ros2 service call /instrument/generate_pattern "
        "spike_workshop_interfaces/srv/GeneratePattern "
        "\"{"
        "template_name: 'score_4bar', "
        f"output_name: '{output_name}', "
        f"output_dir: '{output_dir}', "
        f"speed: {speed}, "
        "duration_sec: 0.0, "
        f"gap_sec: {gap_sec}, "
        f"repeats: {repeats}, "
        f"bpm: {bpm}, "
        "degrees: 0, "
        f"motor_score: '{motor_score}', "
        f"melody_score: '{melody_score}', "
        f"beep_volume: {beep_volume}"
        "}\""
    )


def _print_patterns(message: str) -> None:
    lines = [line.strip() for line in message.splitlines() if line.strip()]
    if not lines:
        print("No patterns found.")
        return
    print("Available patterns:")
    for item in lines:
        print(f"  - {item}")


def _select_score_source() -> Tuple[str, str]:
    preset_names = tuple(SCORE_PRESETS.keys())
    choice = _ask_choice(
        "Score source",
        ("preset", "custom"),
        "preset",
    )

    if choice == "custom":
        print("Motor score format: 4 bars with 4 beats (F/B/S), e.g.")
        print("  F S F S | F S F S | B S B S | B S B S")
        motor_score = _ask_text("Motor score", SCORE_REST_MOTOR)

        print("Melody score format: notes/freq/rest '-', e.g.")
        print("  C4 D4 E4 F4 | G4 A4 B4 C5 | - - G4 - | C5 - - -")
        melody_score = _ask_text("Melody score", SCORE_REST_MELODY)
        return motor_score, melody_score

    print("Available score presets:")
    for key in preset_names:
        print(f"  - {key}: {SCORE_PRESETS[key]['description']}")
    picked = _ask_choice("Preset name", preset_names, "call_response")
    motor_score = SCORE_PRESETS[picked]["motor_score"]
    melody_score = SCORE_PRESETS[picked]["melody_score"]

    overlay_mode = _ask_choice(
        "Melody overlay",
        ("keep", "none", "preset", "custom"),
        "keep",
    )
    if overlay_mode == "none":
        melody_score = SCORE_REST_MELODY
    elif overlay_mode == "preset":
        melody_key = _ask_choice(
            "Melody preset",
            tuple(MELODY_OVERLAYS.keys()),
            "scale",
        )
        melody_score = MELODY_OVERLAYS[melody_key]
    elif overlay_mode == "custom":
        melody_score = _ask_text("Custom melody score", melody_score)

    return motor_score, melody_score


def _generate_score_flow(client: PatternMenuClient) -> None:
    print("4-bar score builder (4/4, 16 beats per loop)")
    output_name = _ask_text("Output name", "score_custom")
    output_dir = _ask_text("Output dir", "/patterns/user")
    bpm = _ask_float("BPM", 120.0)
    repeats = _ask_int("Loop repeats", 2)
    speed = _ask_float("Motor speed magnitude [0..1]", 0.5)
    beep_volume = _ask_int("Beep volume [0..100]", 60)
    gap_sec = _ask_float("Gap sec between beats", 0.0)

    motor_score, melody_score = _select_score_source()

    ok, message, written_path = client.generate_score(
        output_name=output_name,
        output_dir=output_dir,
        speed=speed,
        gap_sec=gap_sec,
        repeats=repeats,
        bpm=bpm,
        motor_score=motor_score,
        melody_score=melody_score,
        beep_volume=beep_volume,
    )
    if ok:
        print(f"GENERATED: {written_path}")
        print(f"DETAIL: {message}")
    else:
        print(f"ERROR: {message}")

    print("Equivalent command:")
    print(
        _fmt_service_call_generate_score(
            output_name=output_name,
            output_dir=output_dir,
            speed=speed,
            gap_sec=gap_sec,
            repeats=repeats,
            bpm=bpm,
            motor_score=motor_score,
            melody_score=melody_score,
            beep_volume=beep_volume,
        )
    )

    if ok and _ask_yes_no("Play this generated score now?", default_yes=True):
        play_name = output_name
        ok_play, message_play = client.play(
            pattern_name=play_name,
            pattern_path="",
        )
        if ok_play:
            print(f"PLAYING: {message_play}")
        else:
            print(f"ERROR: {message_play}")
        print("Equivalent command:")
        print(_fmt_service_call_play(pattern_name=play_name, pattern_path=""))


def _run_menu() -> None:
    client = PatternMenuClient()
    try:
        print("pattern_menu ready (ROS-first score workflow). Services:")
        print("  /instrument/list_patterns")
        print("  /instrument/generate_pattern")
        print("  /instrument/play_pattern")
        print("  /instrument/stop")

        while True:
            print("")
            print("Select action:")
            print("  1) List patterns")
            print("  2) Play pattern")
            print("  3) Build 4-bar score")
            print("  4) Stop playback")
            print("  5) Exit")
            choice = _ask_text("Choice", "1")

            if choice == "1":
                ok, message = client.list_patterns()
                if not ok:
                    print(f"ERROR: {message}")
                    continue
                _print_patterns(message)
                print("Equivalent command:")
                print('ros2 service call /instrument/list_patterns std_srvs/srv/Trigger "{}"')
                continue

            if choice == "2":
                pattern_name = _ask_text("Pattern name (blank to use pattern_path)", "")
                pattern_path = ""
                if not pattern_name:
                    pattern_path = _ask_text("Pattern path", "/patterns/presets/score_call_response.yaml")
                ok, message = client.play(pattern_name=pattern_name, pattern_path=pattern_path)
                if ok:
                    print(f"PLAYING: {message}")
                else:
                    print(f"ERROR: {message}")
                print("Equivalent command:")
                print(_fmt_service_call_play(pattern_name=pattern_name, pattern_path=pattern_path))
                continue

            if choice == "3":
                _generate_score_flow(client)
                continue

            if choice == "4":
                ok, message = client.stop()
                if ok:
                    print(f"STOPPED: {message}")
                else:
                    print(f"ERROR: {message}")
                print("Equivalent command:")
                print('ros2 service call /instrument/stop std_srvs/srv/Trigger "{}"')
                continue

            if choice == "5":
                print("Exiting pattern_menu.")
                return

            print("Unknown option. Choose 1-5.")
    finally:
        client.close()


def main() -> None:
    rclpy.init()
    try:
        _run_menu()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
