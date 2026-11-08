#!/usr/bin/env bash
# Usage: source env.sh   (from any directory)
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$REPO_DIR/.venv/bin/activate"

if [ -f "$REPO_DIR/ros2_ws/install/setup.bash" ]; then
  source "$REPO_DIR/ros2_ws/install/setup.bash"
fi
