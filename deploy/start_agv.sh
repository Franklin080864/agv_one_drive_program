#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run with sudo because configuring SocketCAN requires root." >&2
  exit 2
fi
if [[ $# -ne 1 ]]; then
  echo "usage: sudo $0 /absolute/path/to/chassis.yaml" >&2
  exit 2
fi

interface="${CAN_INTERFACE:-can0}"
bitrate=1000000
txqueuelen=4096
session_name=agv_control
real_user="${SUDO_USER:-$(id -un)}"
real_home="$(getent passwd "$real_user" | cut -d: -f6)"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws_dir="$(cd "$repo_dir/../.." && pwd)"
setup_file="$repo_dir/deploy/setup_agv_env.bash"
params_file="$(realpath "$1")"

if [[ ! -r "$params_file" ]]; then
  echo "Unreadable parameter file: $params_file" >&2
  exit 2
fi
if [[ ! -r "$ws_dir/install_release_humble/setup.bash" ]]; then
  echo "Workspace is not built. Run: $repo_dir/deploy/build_humble.sh" >&2
  exit 2
fi
if [[ ! -x /usr/bin/tmux ]]; then
  echo "Missing /usr/bin/tmux; run deploy/install_dependencies_ubuntu22_humble.sh" >&2
  exit 2
fi

echo "Keep the physical E-stop PRESSED while the stack starts."
while [[ ! -d "/sys/class/net/$interface" ]]; do
  sleep 1
done
sleep 3

ip link set "$interface" down
ip link set "$interface" type can bitrate "$bitrate"
ip link set "$interface" txqueuelen "$txqueuelen"
ip link set "$interface" up

run_as_user=(sudo -u "$real_user" env HOME="$real_home")
"${run_as_user[@]}" /usr/bin/tmux kill-session -t "$session_name" 2>/dev/null || true
"${run_as_user[@]}" /usr/bin/tmux new-session -d -s "$session_name" /bin/bash

setup_q="$(printf '%q' "$setup_file")"
params_q="$(printf '%q' "$params_file")"
interface_q="$(printf '%q' "$interface")"
init_cmd="source $setup_q"

"${run_as_user[@]}" /usr/bin/tmux send-keys -t "$session_name" \
  "$init_cmd && ros2 launch ros2_socketcan socket_can_receiver.launch.py interface:=$interface_q" C-m

"${run_as_user[@]}" /usr/bin/tmux split-window -v -t "$session_name" /bin/bash
"${run_as_user[@]}" /usr/bin/tmux send-keys -t "$session_name" \
  "$init_cmd && ros2 launch ros2_socketcan socket_can_sender.launch.py interface:=$interface_q" C-m

"${run_as_user[@]}" /usr/bin/tmux select-pane -t 0
"${run_as_user[@]}" /usr/bin/tmux split-window -h -t "$session_name" /bin/bash
"${run_as_user[@]}" /usr/bin/tmux send-keys -t "$session_name" \
  "$init_cmd && ros2 run joy joy_node" C-m

"${run_as_user[@]}" /usr/bin/tmux select-pane -t 2
"${run_as_user[@]}" /usr/bin/tmux split-window -h -t "$session_name" /bin/bash
"${run_as_user[@]}" /usr/bin/tmux send-keys -t "$session_name" \
  "$init_cmd && ros2 launch agv2_pkg agv2_control.launch.py params_file:=$params_q" C-m

"${run_as_user[@]}" /usr/bin/tmux select-layout -t "$session_name" tiled

echo "AGV stack started with explicit parameters: $params_file"
echo "View: tmux attach -t $session_name"
