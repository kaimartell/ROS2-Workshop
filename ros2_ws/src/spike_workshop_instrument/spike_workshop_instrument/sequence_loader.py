from pathlib import Path
from typing import Dict, List, Tuple

import yaml


def load_sequence_file(sequence_file: str) -> Tuple[str, List[Dict[str, float]]]:
    if not sequence_file:
        raise ValueError("sequence_file must be set when mode=sequence")

    path = Path(sequence_file)
    if not path.is_file():
        raise ValueError(f"Sequence file not found: {sequence_file}")

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    name = str(data.get("name", path.stem))
    sequence = data.get("sequence")

    if not isinstance(sequence, list) or not sequence:
        raise ValueError("Sequence YAML must contain a non-empty 'sequence' list")

    steps: List[Dict[str, float]] = []
    for index, step in enumerate(sequence, start=1):
        if not isinstance(step, dict):
            raise ValueError(f"Invalid step at index {index}: expected mapping")

        try:
            speed = float(step.get("speed", 0.0))
            duration = float(step.get("duration", 0.0))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid speed/duration in step {index}: {exc}") from exc

        if duration < 0.0:
            raise ValueError(f"Duration must be >= 0 in step {index}")

        steps.append({"speed": speed, "duration": duration})

    return name, steps
