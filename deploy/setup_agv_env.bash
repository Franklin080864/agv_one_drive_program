#!/usr/bin/env bash

# Source this file. It intentionally preserves the caller's nounset setting
# because ROS 2 Humble setup scripts may inspect unset AMENT variables.
# Keep an explicitly selected per-robot domain across both ROS setup files.
_agv_domain_was_set="${ROS_DOMAIN_ID+x}"
_agv_requested_domain="${ROS_DOMAIN_ID-}"
if [[ -n "$_agv_domain_was_set" ]] && {
  [[ ! "$_agv_requested_domain" =~ ^[0-9]{1,3}$ ]] || ((10#$_agv_requested_domain > 232));
}; then
  echo "Invalid ROS_DOMAIN_ID: expected an integer from 0 to 232." >&2
  unset _agv_domain_was_set _agv_requested_domain
  return 2
fi
case $- in
  *u*) _agv_restore_nounset=1 ;;
  *) _agv_restore_nounset=0 ;;
esac
set +u

_agv_repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_agv_ws_dir="$(cd "$_agv_repo_dir/../.." && pwd)"

_agv_setup_status=0
source /opt/ros/humble/setup.bash || _agv_setup_status=$?
if [[ $_agv_setup_status -eq 0 ]]; then
  source "$_agv_ws_dir/install_release_humble/setup.bash" || _agv_setup_status=$?
fi
if [[ -n "$_agv_domain_was_set" ]]; then
  export ROS_DOMAIN_ID="$((10#$_agv_requested_domain))"
fi

if [[ $_agv_restore_nounset -eq 1 ]]; then
  set -u
fi
unset _agv_restore_nounset _agv_repo_dir _agv_ws_dir _agv_domain_was_set _agv_requested_domain
if [[ $_agv_setup_status -ne 0 ]]; then
  unset _agv_setup_status
  return 1
fi
unset _agv_setup_status
