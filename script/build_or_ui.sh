#!/usr/bin/env bash
set -Eeuo pipefail

# Config
WS="${HOME}/ros2_ws"
PKG_MSG="omnirevolve_ros2_messages"
PKG_UI="omnirevolve_ros2_ui"
EXEC_UI="omnirevolve_ros2_ui"

# Helpers
usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --ws PATH      ROS 2 workspace path (default: ${WS})
  --clean        Remove build/, install/, log/ in the workspace and exit
  --build        Build ${PKG_MSG} and ${PKG_UI}
  --run          Run: ros2 run ${PKG_UI} ${EXEC_UI}
  -h, --help     Show this help

Examples:
  $(basename "$0") --build
  $(basename "$0") --run
  $(basename "$0") --ws ~/my_ws --build --run
EOF
}

# Source a file safely (temporarily disable nounset for setup scripts)
safe_source() {
  local f="$1"
  if [[ -f "$f" ]]; then
    set +u
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    # shellcheck source=/dev/null
    source "$f"
    set -u
  else
    echo "ERROR: file not found: $f" >&2
    return 1
  fi
}

ensure_ws() {
  mkdir -p "${WS}"
}

clean_ws() {
  echo "Cleaning ${WS}/{build,install,log} ..."
  rm -rf "${WS}/build" "${WS}/install" "${WS}/log"
  echo "Clean done."
}

build_msgs() {
  echo "=== Building ${PKG_MSG} ==="
  pushd "${WS}" >/dev/null
  unset AMENT_PREFIX_PATH || true
  unset CMAKE_PREFIX_PATH || true
  safe_source "/opt/ros/humble/setup.bash"
  colcon build --packages-select "${PKG_MSG}" --symlink-install \
    --event-handlers console_direct+ status- \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  popd >/dev/null
}

build_ui() {
  echo "=== Building ${PKG_UI} ==="
  pushd "${WS}" >/dev/null
  unset AMENT_PREFIX_PATH || true
  unset CMAKE_PREFIX_PATH || true
  safe_source "/opt/ros/humble/setup.bash"
  safe_source "${WS}/install/setup.bash"
  colcon build --packages-select "${PKG_UI}" --symlink-install \
    --event-handlers console_direct+ status- \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  popd >/dev/null
}

run_ui() {
  echo "=== Running ${PKG_UI}/${EXEC_UI} ==="
  pushd "${WS}" >/dev/null
  unset AMENT_PREFIX_PATH || true
  unset CMAKE_PREFIX_PATH || true
  safe_source "/opt/ros/humble/setup.bash"
  safe_source "${WS}/install/setup.bash"
  if ! ros2 pkg executables "${PKG_UI}" | grep -qE "^${PKG_UI}[[:space:]]+${EXEC_UI}\$"; then
    echo "ERROR: Executable ${EXEC_UI} not found in package ${PKG_UI}."
    echo "Hint: run build first: $(basename "$0") --build"
    exit 1
  fi
  ros2 run "${PKG_UI}" "${EXEC_UI}"
  popd >/dev/null
}

# Arg parsing
DO_CLEAN=0
DO_BUILD=0
DO_RUN=0

while (( "$#" )); do
  case "$1" in
    --ws)    WS="$2"; shift 2;;
    --clean) DO_CLEAN=1; shift;;
    --build) DO_BUILD=1; shift;;
    --run)   DO_RUN=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1" >&2; usage; exit 1;;
  esac
done

ensure_ws

if [[ $DO_CLEAN -eq 1 ]]; then
  clean_ws
  exit 0
fi

if [[ $DO_BUILD -eq 1 ]]; then
  build_msgs
  build_ui
fi

if [[ $DO_RUN -eq 1 ]]; then
  run_ui
fi

if [[ $DO_BUILD -eq 0 && $DO_RUN -eq 0 ]]; then
  usage
fi
