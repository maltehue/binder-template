#!/usr/bin/env bash
set -euo pipefail

# --------------------------
# Config
# --------------------------
SESH="teleop_garmi"
MUJOCO_VERSION="${MUJOCO_VERSION:-3.5.0}"

DEMO_DIR="/home/jovyan/libs/RIG2026/demo"
VENV_DIR="${DEMO_DIR}/multiverse"
MUJOCO_DIR="${DEMO_DIR}/mujoco-${MUJOCO_VERSION}"
MULTIVERSE_DIR="/home/jovyan/libs/RIG2026/Multiverse"
URDF_ROS2="${DEMO_DIR}/assets/urdf/garmi.urdf"
MJCF_SCENE="${DEMO_DIR}/assets/mjcf/scene_position_with_multiverse.xml"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

COLCON_WS="${MULTIVERSE_DIR}/MultiverseConnector/ros_connector/ros_ws/multiverse_ws2"
ROSPKG_SETUP="${COLCON_WS}/install/setup.bash"

REQ_LOCAL="${DEMO_DIR}/requirements.txt"
REQ_ROOT="requirements.txt"

# --------------------------
# Helpers
# --------------------------
log()  { echo -e "\n\033[1;32m[+] $*\033[0m"; }
warn() { echo -e "\n\033[1;33m[!] $*\033[0m"; }
die()  { echo -e "\n\033[1;31m[✗] $*\033[0m" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

tmux_send() {
  local target="$1"; shift
  tmux send-keys -t "$target" "$*" C-m
}

ros_source() {
  local overlay="${1:-}"
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if [[ -n "$overlay" ]]; then
    # shellcheck disable=SC1090
    source "$overlay"
  fi
  set -u
}

if tmux has-session -t "$SESH" 2>/dev/null; then
  exec tmux attach -t "$SESH"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
cd ..

need_cmd tmux
need_cmd python3
need_cmd wget
need_cmd tar

[[ -f "$ROS_SETUP" ]] || die "ROS setup not found: $ROS_SETUP"

echo "$VENV_DIR"

if [[ ! -d "$VENV_DIR" ]]; then
  log "Creating venv: $VENV_DIR"
  mkdir -p "$(dirname "$VENV_DIR")"
  python3 -m venv "$VENV_DIR"
  # shellcheck disable=SC1090
  source "$VENV_DIR/bin/activate"
  python -m pip install -U pip
  pip install -r "$REQ_LOCAL"
  (cd ${MULTIVERSE_DIR}; pip install -r "$REQ_ROOT"; pip install -e "MultiverseConnector/ros_connector")
else
  log "Using existing venv: $PWD/$VENV_DIR"
  # shellcheck disable=SC1090
  source "$VENV_DIR/bin/activate"
fi

if [[ ! -d "$MUJOCO_DIR" ]]; then
  log "Downloading MuJoCo ${MUJOCO_VERSION} -> ${MUJOCO_DIR}"
  mkdir -p "$DEMO_DIR"
  wget -qO- "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
    | tar -xz -C "$DEMO_DIR"
fi

mkdir -p "${MUJOCO_DIR}/bin/mujoco_plugin"
if compgen -G "${MULTIVERSE_DIR}/MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/*.so" >/dev/null; then
  cp -f "${MULTIVERSE_DIR}/MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/"*.so "${MUJOCO_DIR}/bin/mujoco_plugin/" || true
else
  warn "No plugin .so found at Multiverse/MultiverseConnector/mujoco_connector/mujoco-${MUJOCO_VERSION}/*.so"
fi

ros_source ""

if [[ ! -f "$ROSPKG_SETUP" ]]; then
  log "Building colcon workspace: ${COLCON_WS}"
  need_cmd colcon
  pushd "$COLCON_WS" >/dev/null
  colcon build --symlink-install
  popd >/dev/null
fi

ros_source "$ROSPKG_SETUP"

