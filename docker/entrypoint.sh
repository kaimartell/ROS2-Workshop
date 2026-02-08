#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
if [[ -f /ros2_ws/install/setup.bash ]]; then
  source /ros2_ws/install/setup.bash
fi

if [[ "${AUTO_LAUNCH:-0}" == "1" ]]; then
  if [[ -n "${AUTO_LAUNCH_ARGS:-}" ]]; then
    # shellcheck disable=SC2086
    exec ros2 launch spike_workshop_instrument instrument.launch.py ${AUTO_LAUNCH_ARGS}
  fi
  exec ros2 launch spike_workshop_instrument instrument.launch.py
fi

exec "$@"
