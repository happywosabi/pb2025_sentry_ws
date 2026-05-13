#!/usr/bin/env bash
# One-shot workspace build that sources the env helper and runs colcon.
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${WS_DIR}/scripts/env.sh"
cd "${WS_DIR}"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers "$(nproc)" "$@"
echo
echo "Build done. Source the overlay:"
echo "  source ${WS_DIR}/install/setup.bash"
