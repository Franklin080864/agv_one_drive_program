#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
  echo "Run this script as the deployment user; it will invoke sudo itself." >&2
  exit 2
fi

sudo apt-get update
sudo apt-get install -y \
  git \
  tmux \
  can-utils \
  python3-matplotlib \
  python3-rosdep \
  ros-humble-joy \
  ros-humble-ros2-socketcan

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rosdep install \
  --from-paths "$repo_dir" \
  --ignore-src \
  --rosdistro humble \
  -r -y

echo "Dependency installation complete."
