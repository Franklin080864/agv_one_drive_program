#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run with sudo so can0 can be placed DOWN after shutdown." >&2
  exit 2
fi

interface="${CAN_INTERFACE:-can0}"
session_name=agv_control
real_user="${SUDO_USER:-$(id -un)}"
real_home="$(getent passwd "$real_user" | cut -d: -f6)"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
setup_file="$repo_dir/deploy/setup_agv_env.bash"
setup_q="$(printf '%q' "$setup_file")"
run_as_user=(sudo -u "$real_user" env HOME="$real_home")

echo "The physical E-stop must be PRESSED before shutdown."
if "${run_as_user[@]}" /usr/bin/tmux has-session -t "$session_name" 2>/dev/null; then
  if ! "${run_as_user[@]}" timeout 15s bash -lc \
    "source $setup_q && ros2 service call /agv2/prepare_shutdown std_srvs/srv/Trigger '{}'"; then
    echo "Graceful shutdown handshake failed; tmux and CAN were left running." >&2
    exit 1
  fi
  "${run_as_user[@]}" /usr/bin/tmux kill-session -t "$session_name"
fi

ip link set "$interface" down
ip -details link show "$interface"
echo "Shutdown complete. Confirm state DOWN and CAN state STOPPED above."
