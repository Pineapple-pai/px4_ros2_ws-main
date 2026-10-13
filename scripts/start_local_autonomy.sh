#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(dirname -- "${SCRIPT_DIR}")}"
WAIT_TIMEOUT_S="${WAIT_TIMEOUT_S:-60}"
POSITION_TOPIC="/fmu/out/vehicle_local_position_v1"

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash" >&2
  exit 1
}
[[ -f "${WS_DIR}/install/setup.bash" ]] || {
  echo "Workspace is not built: ${WS_DIR}/install/setup.bash" >&2
  exit 1
}

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "Waiting for PX4 local position topic ${POSITION_TOPIC}..."
elapsed_s=0
until ros2 topic list 2>/dev/null | rg -q "^${POSITION_TOPIC}$"; do
  if ((elapsed_s >= WAIT_TIMEOUT_S)); then
    echo "Timed out waiting for ${POSITION_TOPIC}. Check PX4, MicroXRCE-DDS Agent, and optical-flow EKF configuration." >&2
    exit 2
  fi
  sleep 1
  ((elapsed_s += 1))
done

exec ros2 launch px4_autonomy_bringup local_autonomy.launch.py "$@"
