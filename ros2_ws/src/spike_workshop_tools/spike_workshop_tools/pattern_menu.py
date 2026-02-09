#!/usr/bin/env python3
"""Deprecated helper kept for backwards compatibility."""


def main() -> None:
    print("pattern_menu is deprecated for workshop use.")
    print("Use ROS services directly instead:")
    print(
        "  ros2 service call /instrument/generate_score "
        "spike_workshop_interfaces/srv/GenerateScore \"{...}\""
    )
    print(
        "  ros2 service call /instrument/play_pattern "
        "spike_workshop_interfaces/srv/PlayPattern "
        "\"{pattern_name: 'your_score', pattern_path: ''}\""
    )


if __name__ == "__main__":
    main()
