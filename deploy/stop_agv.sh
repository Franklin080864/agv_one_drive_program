#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run with sudo so the recorded CAN interface can be placed DOWN after shutdown." >&2
  exit 2
fi
session_name="${AGV_SESSION_NAME:-agv_control}"
requested_domain="${ROS_DOMAIN_ID:-}"
requested_interface="${CAN_INTERFACE:-}"
shutdown_timeout="${AGV_SHUTDOWN_TIMEOUT_S:-10}"
while [[ $# -gt 0 ]]; do
  if [[ "$1" == --help || "$1" == -h ]]; then
    echo "usage: sudo $0 [--session NAME] [--domain-id ID] [--interface CAN]"
    echo "Domain and CAN default to the selected session's recorded values; explicit values must match."
    exit 0
  fi
  if [[ $# -lt 2 ]]; then echo "Missing value for $1" >&2; exit 2; fi
  case "$1" in
    --session) session_name="$2" ;;
    --domain-id) requested_domain="$2" ;;
    --interface) requested_interface="$2" ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift 2
done
if [[ ! "$session_name" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo "Invalid AGV session name." >&2
  exit 2
fi
if [[ ! "$shutdown_timeout" =~ ^[1-9][0-9]{0,2}$ ]]; then
  echo "AGV_SHUTDOWN_TIMEOUT_S must be an integer from 1 to 999." >&2
  exit 2
fi
real_user="${SUDO_USER:-$(id -un)}"
real_home="$(getent passwd "$real_user" | cut -d: -f6)"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
setup_file="$repo_dir/deploy/setup_agv_env.bash"
client_file="$repo_dir/scripts/prepare_chassis_shutdown.py"
run_as_user=(sudo -u "$real_user" env HOME="$real_home")
tmux_cmd=("${run_as_user[@]}" /usr/bin/tmux)
exec 9>/run/lock/agv-stack.lock
if ! flock -n 9; then
  echo "Another AGV start/stop operation is in progress; no hardware was changed." >&2
  exit 1
fi
if ! "${tmux_cmd[@]}" has-session -t "=$session_name" 2>/dev/null; then
  echo "No session '$session_name' found. Cannot verify robot identity; CAN was left unchanged." >&2
  exit 1
fi
managed="$("${tmux_cmd[@]}" show-options -qv -t "=$session_name" @agv_managed)"
domain_id="$("${tmux_cmd[@]}" show-options -qv -t "=$session_name" @agv_domain_id)"
interface="$("${tmux_cmd[@]}" show-options -qv -t "=$session_name" @agv_can_interface)"
if [[ "$managed" != 1 || ! "$domain_id" =~ ^[0-9]{1,3}$ || ! "$interface" =~ ^[a-zA-Z0-9_-]{1,15}$ ]] || ((10#$domain_id > 232)); then
  echo "Session lacks valid AGV domain/CAN metadata. Refusing to guess; tmux and CAN were left running." >&2
  exit 1
fi
domain_id="$((10#$domain_id))"
if [[ -n "$requested_domain" ]]; then
  if [[ ! "$requested_domain" =~ ^[0-9]{1,3}$ ]] || ((10#$requested_domain != domain_id)); then
    echo "Requested ROS_DOMAIN_ID '$requested_domain' does not match session '$session_name' domain '$domain_id'. No action taken." >&2
    exit 1
  fi
fi
if [[ -n "$requested_interface" && "$requested_interface" != "$interface" ]]; then
  echo "Requested CAN_INTERFACE '$requested_interface' does not match session '$session_name' interface '$interface'. No action taken." >&2
  exit 1
fi
link_type=""
if [[ ! -r "/sys/class/net/$interface/type" ]] ||
   ! read -r link_type < "/sys/class/net/$interface/type" || [[ "$link_type" != 280 ]]; then
  echo "Recorded interface '$interface' is not a verified CAN interface (ARPHRD_CAN=280). No action taken; session metadata was retained." >&2
  exit 1
fi

echo "The physical E-stop must be PRESSED before shutdown."
echo "Stopping session=$session_name ROS_DOMAIN_ID=$domain_id CAN_INTERFACE=$interface"
if ! "${run_as_user[@]}" env ROS_DOMAIN_ID="$domain_id" timeout "$((shutdown_timeout + 5))s" /bin/bash -c \
  'source "$1" && exec python3 "$2" --timeout "$3"' \
  bash "$setup_file" "$client_file" "$shutdown_timeout"; then
  echo "Graceful shutdown handshake failed; tmux and CAN were left running. Keep physical E-stop pressed and inspect the reported error." >&2
  exit 1
fi
# Retain instance identity until CAN is confirmed DOWN. The service is
# idempotent, so a failed interface operation can be retried on the same session.
if ! ip link set "$interface" down; then
  echo "Failed to place '$interface' DOWN; session and domain/CAN metadata were retained for retry." >&2
  exit 1
fi
if ! ip -details link show "$interface"; then
  echo "Could not inspect '$interface' after DOWN; session metadata was retained for retry." >&2
  exit 1
fi
if ! "${tmux_cmd[@]}" kill-session -t "=$session_name"; then
  echo "CAN '$interface' is DOWN but session shutdown failed; retry with the same session/domain." >&2
  exit 1
fi
echo "Shutdown complete for domain $domain_id. Confirm state DOWN and CAN state STOPPED above."
