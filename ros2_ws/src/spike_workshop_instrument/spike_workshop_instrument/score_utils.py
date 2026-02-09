from __future__ import annotations

import re
from typing import Dict, List, Optional, Tuple


SCORE_BEATS = 16

MOTOR_PRESETS_REST = "S S S S | S S S S | S S S S | S S S S"
MELODY_PRESETS_REST = "- - - - | - - - - | - - - - | - - - -"

SCORE_PRESETS: Dict[str, Dict[str, str]] = {
    "motor_basic": {
        "motor_score": "F S F S | F S F S | B S B S | B S B S",
        "melody_score": MELODY_PRESETS_REST,
    },
    "melody_scale": {
        "motor_score": MOTOR_PRESETS_REST,
        "melody_score": "C4 D4 E4 F4 | G4 A4 B4 C5 | C5 B4 A4 G4 | F4 E4 D4 C4",
    },
    "call_response": {
        "motor_score": "F F S S | B B S S | F F S S | B B S S",
        "melody_score": "C4 E4 - - | G4 E4 - - | C5 B4 - - | A4 G4 - -",
    },
    "back_and_forth_groove": {
        "motor_score": "F B F B | F B F B | B F B F | B F B F",
        "melody_score": "A4 - A4 - | C5 - C5 - | G4 - G4 - | E4 - E4 -",
    },
}

MOTOR_REMAP = {
    "F": "F",
    "FORWARD": "F",
    "+": "F",
    "B": "B",
    "BACK": "B",
    "BACKWARD": "B",
    "REV": "B",
    "REVERSE": "B",
    "S": "S",
    "STOP": "S",
    "REST": "S",
    "R": "S",
    "0": "S",
    "-": "S",
}

NOTE_OFFSETS = {
    "C": 0,
    "D": 2,
    "E": 4,
    "F": 5,
    "G": 7,
    "A": 9,
    "B": 11,
}

NOTE_TOKEN_RE = re.compile(r"^([A-Ga-g])([#b]?)(-?\d)$")


def resolve_score_preset(name: str) -> Optional[Tuple[str, str]]:
    key = str(name or "").strip().lower()
    if key not in SCORE_PRESETS:
        return None
    preset = SCORE_PRESETS[key]
    return preset["motor_score"], preset["melody_score"]


def list_score_presets() -> List[str]:
    return sorted(SCORE_PRESETS.keys())


def parse_motor_score(score: str) -> List[str]:
    raw = str(score or "").strip()
    if not raw:
        return ["S"] * SCORE_BEATS

    tokens: List[str] = []
    for bar in _split_bars(raw):
        tokens.extend(_tokenize_motor_bar(bar))

    normalized: List[str] = []
    for idx, token in enumerate(tokens, start=1):
        key = token.strip().upper()
        mapped = MOTOR_REMAP.get(key)
        if mapped is None:
            raise ValueError(
                f"motor_score token #{idx}='{token}' invalid; use F/B/S."
            )
        normalized.append(mapped)

    if len(normalized) < SCORE_BEATS:
        normalized.extend(["S"] * (SCORE_BEATS - len(normalized)))
    return normalized[:SCORE_BEATS]


def parse_melody_score(score: str) -> List[Optional[int]]:
    raw = str(score or "").strip()
    if not raw:
        return [None] * SCORE_BEATS

    tokens: List[str] = []
    for bar in _split_bars(raw):
        tokens.extend(_tokenize_melody_bar(bar))

    resolved: List[Optional[int]] = []
    for idx, token in enumerate(tokens, start=1):
        value = _parse_note_or_freq(token)
        if value is None:
            resolved.append(None)
            continue
        if value < 50 or value > 5000:
            raise ValueError(
                f"melody_score token #{idx}='{token}' resolves to {value}Hz; expected 50..5000."
            )
        resolved.append(value)

    if len(resolved) < SCORE_BEATS:
        resolved.extend([None] * (SCORE_BEATS - len(resolved)))
    return resolved[:SCORE_BEATS]


def _split_bars(score: str) -> List[str]:
    bars = [part.strip() for part in str(score).split("|")]
    bars = [bar for bar in bars if bar]
    if not bars:
        return [""]
    return bars[:4]


def _tokenize_motor_bar(bar: str) -> List[str]:
    raw = bar.strip()
    if not raw:
        return []

    if any(ch.isspace() for ch in raw) or "," in raw:
        return [item for item in re.split(r"[,\s]+", raw) if item]

    compact = raw.replace("-", "")
    if compact and all(ch.upper() in {"F", "B", "S"} for ch in compact):
        return [ch for ch in compact]

    if "-" in raw:
        return [item for item in raw.split("-") if item]

    return [raw]


def _tokenize_melody_bar(bar: str) -> List[str]:
    raw = bar.strip()
    if not raw:
        return []

    if any(ch.isspace() for ch in raw) or "," in raw:
        return [item for item in re.split(r"[,\s]+", raw) if item]

    if "-" in raw:
        parts = raw.split("-")
        result: List[str] = []
        for part in parts:
            item = part.strip()
            if item:
                result.append(item)
            else:
                result.append("-")
        return result

    return [raw]


def _parse_note_or_freq(token: str) -> Optional[int]:
    raw = str(token or "").strip()
    if not raw:
        return None

    upper = raw.upper()
    if upper in {"-", "REST", "R"}:
        return None

    try:
        return int(round(float(raw)))
    except ValueError:
        pass

    match = NOTE_TOKEN_RE.match(raw)
    if not match:
        raise ValueError(
            f"unsupported melody token '{token}'; use note names (e.g. C4, A#4) or frequency Hz."
        )

    note = match.group(1).upper()
    accidental = match.group(2)
    octave = int(match.group(3))
    if octave < 0 or octave > 8:
        raise ValueError(f"octave out of range in '{token}'; expected 0..8.")

    semitone = NOTE_OFFSETS[note]
    if accidental == "#":
        semitone += 1
    elif accidental == "b":
        semitone -= 1

    midi = (octave + 1) * 12 + semitone
    freq = int(round(440.0 * (2.0 ** ((midi - 69) / 12.0))))
    return freq
