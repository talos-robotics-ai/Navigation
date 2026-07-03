#!/usr/bin/env bash
# ==========================================================================
# Navigation stack — native Jetson setup (no Docker)
#
# JetPack 6.2 (L4T R36.4.7) · ROS 2 Humble · CUDA 12.6 · aarch64
#
# Three components:
#   1. System deps   – missing apt / ROS packages (safe: apt won't downgrade)
#   2. SDKs          – Livox-SDK2, unitree_sdk2, unitree_ros2 → /opt/navigation/
#   3. AMO venv      – Python venv with torch-GPU, cyclonedds, unitree_sdk2py
#
# Usage:
#   chmod +x setup_jetson.sh
#   sudo ./setup_jetson.sh          # installs apt pkgs + builds SDKs
#   ./setup_jetson.sh --venv-only   # (re)creates the AMO Python venv (no sudo)
#
# The script is idempotent — safe to re-run.
# ==========================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_PREFIX="/opt/navigation"
VENV_DIR="${SCRIPT_DIR}/amo/.venv"
NPROC="$(nproc)"

# Jetson PyTorch wheel for JetPack 6 / CUDA 12.6
# https://developer.nvidia.com/embedded/downloads → PyTorch for Jetson
TORCH_JP6_URL="https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl"

log()  { echo -e "\n\033[1;32m[setup]\033[0m $*"; }
warn() { echo -e "\n\033[1;33m[warn]\033[0m $*"; }
err()  { echo -e "\n\033[1;31m[error]\033[0m $*" >&2; exit 1; }

# ──────────────────────────────────────────────────────────────────────────────
# 1. System packages (needs sudo)
# ──────────────────────────────────────────────────────────────────────────────
install_system_deps() {
    log "Installing missing system packages …"

    # ROS packages the Dockerfiles need that aren't already on this Jetson
    local ros_pkgs=(
        ros-humble-rmw-cyclonedds-cpp
        ros-humble-rosidl-generator-dds-idl
        ros-humble-domain-bridge
        ros-humble-joint-state-publisher
        ros-humble-joint-state-publisher-gui
        ros-humble-xacro
        ros-humble-joy-teleop
    )

    # C/C++ dev libs required by DLIO / Livox / Unitree builds
    local dev_pkgs=(
        libgoogle-glog-dev
        libgflags-dev
        libomp-dev
        python3-rosdep
        python3-venv
        # pyzmq: g1_sim_bridge cmd_vel_to_sonic_node PUBs to the SONIC deploy
        # controller over ZMQ (gait:=sonic). Runtime dep for that node.
        python3-zmq
    )

    apt-get update
    apt-get install -y --no-install-recommends "${ros_pkgs[@]}" "${dev_pkgs[@]}"
    rm -rf /var/lib/apt/lists/*

    # rosdep init (idempotent)
    rosdep init 2>/dev/null || true
    sudo -u "${SUDO_USER:-dev}" rosdep update || true
}

# ──────────────────────────────────────────────────────────────────────────────
# 2a. Livox-SDK2 (needed by livox_ros_driver2 in ros2_ws)
# ──────────────────────────────────────────────────────────────────────────────
build_livox_sdk2() {
    if [ -f "${SDK_PREFIX}/lib/liblivox_lidar_sdk_shared.so" ]; then
        log "Livox-SDK2 already installed — skipping."
        return
    fi
    log "Building Livox-SDK2 → ${SDK_PREFIX} …"
    local src="/tmp/Livox-SDK2"
    rm -rf "${src}"
    git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "${src}"
    cmake -S "${src}" -B "${src}/build" \
        -DCMAKE_INSTALL_PREFIX="${SDK_PREFIX}" \
        -DCMAKE_BUILD_TYPE=Release
    cmake --build "${src}/build" -j"${NPROC}"
    cmake --install "${src}/build"
    # Ensure shared lib is present
    cp -f "${src}/build/sdk_core/liblivox_lidar_sdk_shared.so" "${SDK_PREFIX}/lib/" 2>/dev/null || true
    ldconfig
    rm -rf "${src}"
}

# ──────────────────────────────────────────────────────────────────────────────
# 2b. Unitree SDK2 (C++ SDK for G1)
# ──────────────────────────────────────────────────────────────────────────────
build_unitree_sdk2() {
    if [ -d "${SDK_PREFIX}/unitree_sdk2" ] && [ -f "${SDK_PREFIX}/unitree_sdk2/build/libunitree_sdk2.a" ]; then
        log "Unitree SDK2 already built — skipping."
        return
    fi
    log "Building Unitree SDK2 → ${SDK_PREFIX}/unitree_sdk2 …"
    local dst="${SDK_PREFIX}/unitree_sdk2"
    rm -rf "${dst}"
    git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git "${dst}"
    mkdir -p "${dst}/build"
    cmake -S "${dst}" -B "${dst}/build"
    cmake --build "${dst}/build" -j"${NPROC}"
    # install into system
    cmake --install "${dst}/build" 2>/dev/null || true
    ldconfig
}

# ──────────────────────────────────────────────────────────────────────────────
# 2c. Unitree ROS 2 (msg packages + CycloneDDS workspace)
# ──────────────────────────────────────────────────────────────────────────────
build_unitree_ros2() {
    if [ -d "${SDK_PREFIX}/unitree_ros2/cyclonedds_ws/install" ]; then
        log "Unitree ROS2 workspaces already built — skipping."
        return
    fi
    log "Building Unitree ROS2 packages → ${SDK_PREFIX}/unitree_ros2 …"
    local dst="${SDK_PREFIX}/unitree_ros2"
    rm -rf "${dst}"
    git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2.git "${dst}"

    # CycloneDDS workspace
    (
        source /opt/ros/humble/setup.bash
        cd "${dst}/cyclonedds_ws"
        rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true
        colcon build --symlink-install
    )

    # Example / msg workspace
    (
        source /opt/ros/humble/setup.bash
        source "${dst}/cyclonedds_ws/install/setup.bash"
        cd "${dst}/example"
        rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true
        colcon build --symlink-install
    )
}

# ──────────────────────────────────────────────────────────────────────────────
# 3. AMO policy Python venv (no sudo needed)
# ──────────────────────────────────────────────────────────────────────────────
setup_amo_venv() {
    log "Creating AMO policy venv → ${VENV_DIR} …"

    python3 -m venv --system-site-packages "${VENV_DIR}"
    source "${VENV_DIR}/bin/activate"

    pip install --upgrade pip wheel setuptools

    # ── PyTorch (NVIDIA Jetson wheel, CUDA-enabled) ──
    if python -c "import torch; assert torch.cuda.is_available()" 2>/dev/null; then
        log "torch with CUDA already in venv — skipping."
    else
        log "Installing Jetson-optimized PyTorch (CUDA 12.6) …"
        pip install "${TORCH_JP6_URL}"
    fi

    # ── Eclipse Cyclone DDS C library (build from source) ──
    if [ ! -f /usr/local/lib/libddsc.so ] && [ ! -f "${SDK_PREFIX}/lib/libddsc.so" ]; then
        log "Building Eclipse Cyclone DDS C library …"
        local src="/tmp/cyclonedds-src"
        rm -rf "${src}"
        git clone --depth 1 --branch releases/0.10.x \
            https://github.com/eclipse-cyclonedds/cyclonedds.git "${src}"
        cmake -S "${src}" -B "${src}/build" \
            -DCMAKE_INSTALL_PREFIX=/usr/local \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_IDLC=ON \
            -DBUILD_EXAMPLES=OFF \
            -DBUILD_TESTING=OFF
        cmake --build "${src}/build" --target install -j"${NPROC}"
        rm -rf "${src}"
        ldconfig
    fi
    export CYCLONEDDS_HOME=/usr/local

    # ── Python dependencies (mirrors Dockerfile.amo_policy) ──
    pip install \
        numpy \
        scipy \
        onnxruntime \
        joblib \
        pyyaml \
        easydict \
        python-box \
        tqdm \
        pyzmq \
        pydantic \
        mujoco \
        msgpack \
        msgpack-numpy \
        colorlog \
        websockets \
        matplotlib

    # ── unitree_sdk2py (with CRC lib fix) ──
    log "Installing unitree_sdk2py …"
    local usdk="/tmp/unitree_sdk2_python"
    rm -rf "${usdk}"
    git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git "${usdk}"
    # Fix missing __init__.py in subpackages
    find "${usdk}/unitree_sdk2py" -mindepth 1 -maxdepth 1 -type d \
        -exec sh -c 'test -f "$1/__init__.py" || touch "$1/__init__.py"' _ {} \;
    pip install "${usdk}"
    # Fix CRC native lib not included in wheel
    local site_pkg
    site_pkg="$(cd /tmp && python -c 'import unitree_sdk2py, os; print(os.path.dirname(unitree_sdk2py.__file__))')"
    mkdir -p "${site_pkg}/utils/lib"
    cp "${usdk}/unitree_sdk2py/utils/lib/"*.so "${site_pkg}/utils/lib/" 2>/dev/null || true
    rm -rf "${usdk}"

    # ── Smoke test ──
    log "Smoke-testing AMO venv imports …"
    python - <<'PY'
import importlib
for mod in ("torch", "scipy", "numpy", "mujoco", "pydantic", "box",
            "msgpack", "msgpack_numpy", "colorlog", "websockets",
            "unitree_sdk2py", "cyclonedds"):
    importlib.import_module(mod)
    print(f"  ✓ {mod}")
import torch
print(f"  torch CUDA: {torch.cuda.is_available()}")
from unitree_sdk2py.utils.crc import CRC
CRC()
print("  ✓ CRC native lib loaded")
PY

    deactivate
    log "AMO venv ready. Activate with:  source ${VENV_DIR}/bin/activate"
}

# ──────────────────────────────────────────────────────────────────────────────
# 4. Convenience shell helpers
# ──────────────────────────────────────────────────────────────────────────────
install_helpers() {
    log "Installing shell helpers …"

    # build_ws — same as Dockerfile.localization's helper
    cat > /usr/local/bin/build_ws <<'SCRIPT'
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
WS="${1:-/home/dev/Navigation/ros2_ws}"
cd "${WS}"
export LIVOX_SDK2_ROOT=/opt/navigation
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true
colcon build --symlink-install \
    --cmake-args -DROS_EDITION=ROS2
echo "[build_ws] done — source ${WS}/install/setup.bash"
SCRIPT
    chmod +x /usr/local/bin/build_ws

    # nav_env — source all the overlays in one shot
    cat > /usr/local/bin/nav_env <<SCRIPT
#!/bin/bash
# Source this file:  source nav_env
source /opt/ros/humble/setup.bash

[ -f /opt/navigation/unitree_ros2/cyclonedds_ws/install/setup.bash ] && \
    source /opt/navigation/unitree_ros2/cyclonedds_ws/install/setup.bash
[ -f /opt/navigation/unitree_ros2/example/install/setup.bash ] && \
    source /opt/navigation/unitree_ros2/example/install/setup.bash
[ -f /home/dev/Navigation/ros2_ws/install/setup.bash ] && \
    source /home/dev/Navigation/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
export LIVOX_SDK2_ROOT=/opt/navigation

echo "[nav_env] ROS 2 Humble + Unitree + Navigation overlays loaded."
SCRIPT
    chmod +x /usr/local/bin/nav_env
}

# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
if [[ "${1:-}" == "--venv-only" ]]; then
    setup_amo_venv
    exit 0
fi

if [[ $EUID -ne 0 ]]; then
    err "System deps + SDK builds require root. Run:  sudo $0\n     Or for venv only:  $0 --venv-only"
fi

mkdir -p "${SDK_PREFIX}"

install_system_deps
build_livox_sdk2
build_unitree_sdk2
build_unitree_ros2
install_helpers

# The venv step doesn't need root — run as the real user
log "Switching to user '${SUDO_USER:-dev}' for AMO venv setup …"
sudo -u "${SUDO_USER:-dev}" bash -c "source $(realpath "$0") --venv-only" || \
    setup_amo_venv

log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log "Setup complete!"
log ""
log "Next steps:"
log "  1. Fix the RoboJuDo symlink:"
log "       ln -sf /path/to/RoboJuDo ${SCRIPT_DIR}/policy/RoboJuDo"
log ""
log "  2. Build the DLIO workspace:"
log "       source nav_env"
log "       build_ws"
log ""
log "  3. Run AMO policy:"
log "       source ${VENV_DIR}/bin/activate"
log "       export PYTHONPATH=${SCRIPT_DIR}/policy/RoboJuDo:\$PYTHONPATH"
log "       python ${SCRIPT_DIR}/amo/amo_inference.py --config ${SCRIPT_DIR}/docker/config/amo_g1.yaml"
log ""
log "  4. Source nav_env in your .bashrc for convenience:"
log "       echo 'source nav_env' >> ~/.bashrc"
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
