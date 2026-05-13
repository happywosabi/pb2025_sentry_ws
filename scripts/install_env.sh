#!/usr/bin/env bash
# =============================================================================
#  pb2025_sentry_ws  one-shot environment installer
#  - Installs system / ROS apt deps
#  - Installs Intel OpenVINO 2024.6 (preferred) via Intel apt repo
#  - Builds & installs koide3/small_gicp from source
#  - Runs rosdep
#  - Writes a setup helper at scripts/env.sh that the build script will source
#
#  Re-runnable: each step is idempotent (skipped if already satisfied).
#  Requires: Ubuntu 22.04 (jammy), ROS 2 Humble already installed, sudo rights.
# =============================================================================
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_PREFIX="\033[1;34m[install_env]\033[0m"
log()  { echo -e "${LOG_PREFIX} $*"; }
warn() { echo -e "\033[1;33m[install_env][warn]\033[0m $*"; }
err()  { echo -e "\033[1;31m[install_env][err ]\033[0m $*" >&2; }

ROS_DISTRO="${ROS_DISTRO:-humble}"
OPENVINO_VERSION="2024.6.0"
OPENVINO_PKG="openvino-${OPENVINO_VERSION}"
OPENVINO_DIR_DEFAULT="/opt/intel/openvino_${OPENVINO_VERSION}"
SMALL_GICP_TAG="master"

# --- 0. sanity ---------------------------------------------------------------
if ! command -v apt-get >/dev/null; then
  err "This script targets Ubuntu (apt-get not found)."
  exit 1
fi
if ! command -v sudo >/dev/null; then
  err "sudo is required."
  exit 1
fi
if [[ ! -d "/opt/ros/${ROS_DISTRO}" ]]; then
  err "ROS 2 ${ROS_DISTRO} not detected at /opt/ros/${ROS_DISTRO}. Install it first."
  exit 1
fi

log "Workspace: ${WS_DIR}"
log "ROS_DISTRO=${ROS_DISTRO}"

# --- 1. base apt deps --------------------------------------------------------
log "Step 1/5 — installing base apt packages"
sudo apt-get update -y
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git curl wget gnupg lsb-release ca-certificates \
  python3-pip python3-rosdep python3-vcstool python3-colcon-common-extensions \
  libeigen3-dev libomp-dev libgoogle-glog-dev libgflags-dev libceres-dev \
  libfmt-dev libspdlog-dev libyaml-cpp-dev nlohmann-json3-dev \
  libusb-1.0-0-dev libopencv-dev libpcl-dev libtbb-dev \
  ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions \
  ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-rclcpp-components ros-${ROS_DISTRO}-example-interfaces \
  ros-${ROS_DISTRO}-behaviortree-cpp \
  ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-nav2-common \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# --- 2. rosdep ---------------------------------------------------------------
log "Step 2/5 — rosdep init/update + install workspace deps"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init || true
fi
rosdep update || true
# Some packages ship system deps (eigen, yaml-cpp) — let rosdep resolve.
# --skip-keys lists things we satisfy manually below or via Intel repo.
rosdep install --from-paths "${WS_DIR}/src" --ignore-src -r -y \
  --rosdistro "${ROS_DISTRO}" \
  --skip-keys "openvino small_gicp" || warn "rosdep had partial errors (continuing)"

# --- 3. OpenVINO 2024.6 ------------------------------------------------------
log "Step 3/5 — Intel OpenVINO ${OPENVINO_VERSION}"
if [[ -f "${OPENVINO_DIR_DEFAULT}/runtime/cmake/OpenVINOConfig.cmake" ]]; then
  log "OpenVINO already present at ${OPENVINO_DIR_DEFAULT}, skipping install."
else
  # Add Intel APT key + repo (idempotent)
  KEYRING=/usr/share/keyrings/intel-openvino.gpg
  if [[ ! -f "${KEYRING}" ]]; then
    log "Adding Intel OpenVINO apt key"
    wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
      | sudo gpg --dearmor -o "${KEYRING}"
  fi
  REPO_LIST=/etc/apt/sources.list.d/intel-openvino-2024.list
  if [[ ! -f "${REPO_LIST}" ]]; then
    log "Adding Intel OpenVINO apt repository"
    echo "deb [signed-by=${KEYRING}] https://apt.repos.intel.com/openvino/2024 ubuntu22 main" \
      | sudo tee "${REPO_LIST}" >/dev/null
    sudo apt-get update -y
  fi
  if apt-cache show "${OPENVINO_PKG}" >/dev/null 2>&1; then
    log "Installing ${OPENVINO_PKG} via apt"
    sudo apt-get install -y "${OPENVINO_PKG}"
  else
    warn "${OPENVINO_PKG} not in apt; falling back to archive download"
    TMPDIR_OV=$(mktemp -d)
    pushd "${TMPDIR_OV}" >/dev/null
    OV_TGZ="l_openvino_toolkit_ubuntu22_${OPENVINO_VERSION}.17404.4c0f47d2335_x86_64.tgz"
    wget -q "https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/${OV_TGZ}"
    sudo mkdir -p /opt/intel
    sudo tar -xzf "${OV_TGZ}" -C /opt/intel
    sudo mv "/opt/intel/$(basename "${OV_TGZ}" .tgz)" "${OPENVINO_DIR_DEFAULT}"
    popd >/dev/null
    rm -rf "${TMPDIR_OV}"
  fi
fi

if [[ ! -f "${OPENVINO_DIR_DEFAULT}/runtime/cmake/OpenVINOConfig.cmake" ]] && \
   [[ ! -f "/usr/lib/cmake/openvino${OPENVINO_VERSION}/OpenVINOConfig.cmake" ]]; then
  err "OpenVINO install verification failed (no OpenVINOConfig.cmake found)"
  exit 1
fi

# Discover the actual OpenVINO_DIR for the env helper.
if [[ -f "${OPENVINO_DIR_DEFAULT}/runtime/cmake/OpenVINOConfig.cmake" ]]; then
  OPENVINO_CMAKE_DIR="${OPENVINO_DIR_DEFAULT}/runtime/cmake"
  OPENVINO_SETUPVARS="${OPENVINO_DIR_DEFAULT}/setupvars.sh"
else
  OPENVINO_CMAKE_DIR="/usr/lib/cmake/openvino${OPENVINO_VERSION}"
  OPENVINO_SETUPVARS=""  # apt install integrates with system paths; no setupvars
fi
log "OpenVINO_DIR detected: ${OPENVINO_CMAKE_DIR}"

# --- 4. small_gicp from source ----------------------------------------------
log "Step 4/5 — small_gicp"
if [[ -f /usr/local/lib/cmake/small_gicp/small_gicpConfig.cmake ]] || \
   [[ -f /usr/lib/cmake/small_gicp/small_gicpConfig.cmake ]]; then
  log "small_gicp already installed system-wide, skipping."
else
  TMPDIR_SG=$(mktemp -d)
  log "Building small_gicp in ${TMPDIR_SG}"
  git clone --depth 1 --branch "${SMALL_GICP_TAG}" \
    https://github.com/koide3/small_gicp.git "${TMPDIR_SG}/small_gicp"
  cmake -S "${TMPDIR_SG}/small_gicp" -B "${TMPDIR_SG}/small_gicp/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_HELPER=ON
  cmake --build "${TMPDIR_SG}/small_gicp/build" -j"$(nproc)"
  sudo cmake --install "${TMPDIR_SG}/small_gicp/build"
  sudo ldconfig
  rm -rf "${TMPDIR_SG}"
fi

# --- 5. write env.sh helper --------------------------------------------------
log "Step 5/5 — writing scripts/env.sh"
cat > "${WS_DIR}/scripts/env.sh" <<EOF
# Auto-generated by install_env.sh — source before colcon build / ros2 launch
set -a
ROS_DISTRO="${ROS_DISTRO}"
OpenVINO_DIR="${OPENVINO_CMAKE_DIR}"
set +a

# ROS underlay
if [[ -f "/opt/ros/\${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/\${ROS_DISTRO}/setup.bash"
fi

# OpenVINO runtime env (only present for archive install, not apt install)
if [[ -n "${OPENVINO_SETUPVARS}" && -f "${OPENVINO_SETUPVARS}" ]]; then
  export INTEL_OPENVINO_DIR="${OPENVINO_DIR_DEFAULT}"
  # shellcheck disable=SC1091
  source "${OPENVINO_SETUPVARS}"
fi

# Make small_gicp + apt-installed OpenVINO findable
export CMAKE_PREFIX_PATH="\${OpenVINO_DIR}:/usr/local:\${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/usr/local/lib:\${LD_LIBRARY_PATH:-}"

# Workspace overlay (only after first successful build)
if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  source "${WS_DIR}/install/setup.bash"
fi
EOF
chmod +x "${WS_DIR}/scripts/env.sh"

log "DONE. Next steps:"
log "  source ${WS_DIR}/scripts/env.sh"
log "  ${WS_DIR}/scripts/build_ws.sh         # one-shot build"
