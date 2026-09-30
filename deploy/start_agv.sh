#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: sudo $0 [--domain-id ID] [--session NAME] [--interface CAN] /absolute/path/to/chassis.yaml" >&2
  echo "Domain is required via --domain-id or sudo env ROS_DOMAIN_ID=ID. Defaults: AGV_SESSION_NAME=agv_control, CAN_INTERFACE=can0." >&2
}
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run with sudo because configuring SocketCAN requires root." >&2
  exit 2
fi

domain_id="${ROS_DOMAIN_ID:-}"
interface="${CAN_INTERFACE:-can0}"
session_name="${AGV_SESSION_NAME:-agv_control}"
startup_timeout="${AGV_STARTUP_TIMEOUT_S:-30}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --domain-id|--session|--interface)
      if [[ $# -lt 2 ]]; then usage; exit 2; fi
      case "$1" in
        --domain-id) domain_id="$2" ;;
        --session) session_name="$2" ;;
        --interface) interface="$2" ;;
      esac
      shift 2 ;;
    --help|-h) usage; exit 0 ;;
    --*) usage; exit 2 ;;
    *) break ;;
  esac
done
if [[ $# -ne 1 ]]; then usage; exit 2; fi
if [[ ! "$domain_id" =~ ^[0-9]{1,3}$ ]] || ((10#$domain_id > 232)); then
  echo "An explicit ROS_DOMAIN_ID from 0 to 232 is required; assign a different ID to each robot." >&2
  exit 2
fi
domain_id="$((10#$domain_id))"
if [[ ! "$session_name" =~ ^[a-zA-Z0-9_-]+$ ]] || [[ ! "$interface" =~ ^[a-zA-Z0-9_-]{1,15}$ ]]; then
  echo "Invalid session/interface name; use letters, digits, underscores or hyphens." >&2
  exit 2
fi
if [[ ! "$startup_timeout" =~ ^[1-9][0-9]{0,2}$ ]]; then
  echo "AGV_STARTUP_TIMEOUT_S must be an integer from 1 to 999." >&2
  exit 2
fi

bitrate=1000000
txqueuelen=4096
real_user="${SUDO_USER:-$(id -un)}"
real_home="$(getent passwd "$real_user" | cut -d: -f6)"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws_dir="$(cd "$repo_dir/../.." && pwd)"
setup_file="$repo_dir/deploy/setup_agv_env.bash"
client_file="$repo_dir/scripts/prepare_chassis_shutdown.py"
params_file="$(realpath "$1")"
if [[ ! -r "$params_file" ]]; then
  echo "Unreadable parameter file: $params_file" >&2
  exit 2
fi
if [[ ! -r "$ws_dir/install_release_humble/setup.bash" ]]; then
  echo "Workspace is not built. Run: $repo_dir/deploy/build_humble.sh" >&2
  exit 2
fi
for program in /usr/bin/tmux sudo flock timeout ip python3; do
  if ! command -v "$program" >/dev/null; then
    echo "Missing command: $program; run deploy/install_dependencies_ubuntu22_humble.sh" >&2
    exit 2
  fi
done
run_as_user=(sudo -u "$real_user" env HOME="$real_home" ROS_DOMAIN_ID="$domain_id")
tmux_cmd=("${run_as_user[@]}" /usr/bin/tmux)

# Serialize transitions, including the check-before-create and stop sequence.
exec 9>/run/lock/agv-stack.lock
if ! flock -n 9; then
  echo "Another AGV start/stop operation is in progress. No hardware was changed." >&2
  exit 1
fi
if "${tmux_cmd[@]}" has-session -t "=$session_name" 2>/dev/null; then
  echo "Session '$session_name' already exists. Stop it explicitly before starting; CAN was left unchanged." >&2
  exit 1
fi
# tmux inventories belong to this deployment user. This is not cross-user
# ownership enforcement; use one deployment account per robot host.
while IFS='|' read -r existing_name existing_interface existing_domain; do
  [[ -n "$existing_name" ]] || continue
  if [[ "$existing_interface" == "$interface" || "$existing_domain" == "$domain_id" ]]; then
    echo "Session '$existing_name' already owns CAN '$existing_interface' / domain '$existing_domain'. Refusing a conflicting instance." >&2
    exit 1
  fi
done < <("${tmux_cmd[@]}" list-sessions -F '#{session_name}|#{@agv_can_interface}|#{@agv_domain_id}' 2>/dev/null || true)

echo "Keep the physical E-stop PRESSED while the stack starts."
echo "Starting session=$session_name ROS_DOMAIN_ID=$domain_id CAN_INTERFACE=$interface"
deadline=$((SECONDS + startup_timeout))
while [[ ! -d "/sys/class/net/$interface" ]]; do
  if ((SECONDS >= deadline)); then
    echo "Timed out waiting for CAN interface '$interface'; no session or hardware was changed." >&2
    exit 1
  fi
  sleep 1
done
link_type=""
if [[ ! -r "/sys/class/net/$interface/type" ]] ||
   ! read -r link_type < "/sys/class/net/$interface/type" || [[ "$link_type" != 280 ]]; then
  echo "Interface '$interface' is not a verified CAN interface (ARPHRD_CAN=280); no hardware was changed." >&2
  exit 1
fi

# Reserve the instance before touching CAN. Metadata is the stop command's source
# of truth; tmux's global server environment may belong to a different robot.
"${tmux_cmd[@]}" new-session -d -s "$session_name" /bin/bash
"${tmux_cmd[@]}" set-option -t "=$session_name" @agv_managed 1
"${tmux_cmd[@]}" set-option -t "=$session_name" @agv_domain_id "$domain_id"
"${tmux_cmd[@]}" set-option -t "=$session_name" @agv_can_interface "$interface"
"${tmux_cmd[@]}" set-option -t "=$session_name" @agv_params_file "$params_file"
"${tmux_cmd[@]}" set-window-option -t "=$session_name" remain-on-exit on
trap 'echo "Startup did not complete. Keep physical E-stop pressed; session and CAN were left available for inspection. No automatic retry/reset was attempted." >&2' ERR

ip link set "$interface" down
ip link set "$interface" type can bitrate "$bitrate"
ip link set "$interface" txqueuelen "$txqueuelen"
ip link set "$interface" up

setup_q="$(printf '%q' "$setup_file")"
init_cmd="export ROS_DOMAIN_ID=$domain_id; source $setup_q && exec"
printf -v receiver_cmd '%q ' ros2 launch ros2_socketcan socket_can_receiver.launch.py "interface:=$interface"
printf -v sender_cmd '%q ' ros2 launch ros2_socketcan socket_can_sender.launch.py "interface:=$interface"
printf -v joy_cmd '%q ' ros2 run joy joy_node
printf -v control_cmd '%q ' ros2 launch agv2_pkg agv2_control.launch.py "params_file:=$params_file"
printf -v receiver_shell '%q ' /bin/bash -c "$init_cmd $receiver_cmd"
printf -v sender_shell '%q ' /bin/bash -c "$init_cmd $sender_cmd"
printf -v joy_shell '%q ' /bin/bash -c "$init_cmd $joy_cmd"
printf -v control_shell '%q ' /bin/bash -c "$init_cmd $control_cmd"

"${tmux_cmd[@]}" respawn-pane -k -t "=$session_name" "$receiver_shell"
"${tmux_cmd[@]}" split-window -d -v -t "=$session_name" "$sender_shell"
"${tmux_cmd[@]}" split-window -d -h -t "=$session_name" "$joy_shell"
"${tmux_cmd[@]}" split-window -d -h -t "=$session_name" "$control_shell"
"${tmux_cmd[@]}" select-layout -t "=$session_name" tiled

# Graph readiness is bounded and cannot enable motion. A failure deliberately
# preserves the evidence and does not tear down CAN while stop is unconfirmed.
if ! "${run_as_user[@]}" timeout "$((startup_timeout + 5))s" /bin/bash -c \
  'source "$1" && exec python3 "$2" --wait-ready --timeout "$3"' \
  bash "$setup_file" "$client_file" "$startup_timeout"; then
  echo "Startup readiness failed. Keep physical E-stop pressed; inspect: tmux attach -t $session_name" >&2
  exit 1
fi
if "${tmux_cmd[@]}" list-panes -t "=$session_name" -F '#{pane_dead}' | grep -q '^1$'; then
  echo "A stack process exited during startup. Keep physical E-stop pressed; inspect session '$session_name'." >&2
  exit 1
fi
trap - ERR
echo "AGV ROS processes ready with parameters: $params_file"
echo "Verify hardware feedback and faults before releasing physical E-stop. View: tmux attach -t $session_name"
