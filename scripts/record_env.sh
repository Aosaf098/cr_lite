#!/usr/bin/env bash
# Record the exact system environment (OS, GPU, Python, ROS 2 apt packages,
# system Python libraries) for reproducibility.
#
# Usage (from the repo root, after `source env.sh`):
#   scripts/record_env.sh > environment/system_versions.txt
#
# The same output can be saved into each experiment run directory later.
set -euo pipefail

echo "# Coverage-Recon Lite — system environment"
echo "# Recorded: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "# Git commit: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo

echo "## OS"
lsb_release -ds
uname -srm
echo

echo "## GPU"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null \
  || echo "nvidia-smi not available"
echo

echo "## Python"
echo "interpreter: $(command -v python)"
python --version
echo

echo "## Python libraries in use (resolved import path and version)"
python - << 'EOF'
import importlib
mods = ["numpy", "scipy", "mujoco", "open3d", "osqp", "matplotlib", "pandas",
        "cv2", "rclpy", "cv_bridge", "pytest"]
for name in mods:
    try:
        m = importlib.import_module(name)
        ver = getattr(m, "__version__", "n/a")
        src = "venv" if "/.venv/" in (m.__file__ or "") else "system"
        print(f"{name:12s} {ver:12s} {src}")
    except Exception as exc:  # noqa: BLE001
        print(f"{name:12s} NOT IMPORTABLE ({type(exc).__name__})")
EOF
echo

echo "## ROS 2"
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo

echo "## apt packages (ros-humble-* and system Python libraries)"
dpkg-query -W -f='${Package}=${Version}\n' \
  'ros-humble-*' python3-opencv python3-numpy python3-pytest \
  python3-colcon-common-extensions python3-rosdep 2>/dev/null | sort
