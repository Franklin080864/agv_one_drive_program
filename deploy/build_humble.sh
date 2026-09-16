#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws_dir="$(cd "$repo_dir/../.." && pwd)"

if [[ "$(basename "$(dirname "$repo_dir")")" != src ]]; then
  echo "Expected repository at <workspace>/src/agv2_pkg; got: $repo_dir" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u

colcon --log-base "$ws_dir/log_release_humble" build \
  --base-paths "$repo_dir" \
  --build-base "$ws_dir/build_release_humble" \
  --install-base "$ws_dir/install_release_humble" \
  --packages-select agv2_pkg \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

ctest \
  --test-dir "$ws_dir/build_release_humble/agv2_pkg" \
  --label-regex gtest \
  --output-on-failure

echo "Build and five functional gtests completed."
echo "Environment: source $repo_dir/deploy/setup_agv_env.bash"
