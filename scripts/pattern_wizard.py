#!/usr/bin/env python3
"""Deprecated local generator kept as a pointer to ROS-first workflow."""


def main() -> int:
    print("pattern_wizard is deprecated for this workshop.")
    print("Use ROS services with the running graph instead:")
    print(
        "  ros2 service call /instrument/generate_score "
        "spike_workshop_interfaces/srv/GenerateScore \"{...}\""
    )
    print(
        "  ros2 service call /instrument/play_pattern "
        "spike_workshop_interfaces/srv/PlayPattern "
        "\"{pattern_name: 'your_score', pattern_path: ''}\""
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
