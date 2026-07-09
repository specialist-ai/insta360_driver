#!/bin/bash
# Glass-to-detect latency comparison across ROS consumer types.
# Run under bash (sources the workspace, which pulls in ROS). Camera must be
# responsive and pointed at a monitor; a fullscreen ArUco window flashes
# during each run.
#
# Runs the two in-package C++ variants (RUN A: separate process, RUN B:
# zero-copy composed). RUN C (a Python rclpy consumer) is optional and needs
# an external harness — set PY_LATENCY_HARNESS to its path to include it.
#
# Env overrides:
#   WS_SETUP           workspace setup.bash (default: ../../../install/setup.bash)
#   INSTA360_BRIDGE    standalone bridge binary for the post-teardown health
#                      check (optional; skipped if unset)
#   PY_LATENCY_HARNESS python rclpy ArUco harness for RUN C (optional)
# (No `set -u`: ROS setup.bash references unbound COLCON_TRACE.)

LOG_DIR="${TMPDIR:-/tmp}/insta360_latency"
mkdir -p "$LOG_DIR"
export DISPLAY="${DISPLAY:-:1}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${WS_SETUP:-$HERE/../../../install/setup.bash}"

CAM_BIN="lib/insta360_camera/camera_node"
CONT_BIN="lib/rclcpp_components/component_container"

echo "== RUN A: C++ full loop, separate processes"
ros2 run insta360_camera camera_node > "$LOG_DIR/runA_cam.log" 2>&1 &
sleep 12
timeout 200 ros2 run insta360_camera aruco_latency_test 2>&1 | grep -E "CPP RESULTS|timeout"
pkill -INT -f "$CAM_BIN"; sleep 10

echo "== RUN B: zero-copy composed"
ros2 run rclcpp_components component_container > "$LOG_DIR/runB.log" 2>&1 &
sleep 4
ros2 component load /ComponentManager insta360_camera insta360_camera::CameraNode -e use_intra_process_comms:=true
sleep 12
ros2 component load /ComponentManager insta360_camera insta360_camera::ArucoLatencyTest -e use_intra_process_comms:=true
for _ in $(seq 1 220); do pgrep -f "$CONT_BIN" > /dev/null || break; sleep 1; done
pgrep -f "$CONT_BIN" > /dev/null && { echo "container hung, TERM"; pkill -TERM -f "$CONT_BIN"; sleep 8; }
grep -E "CPP RESULTS" "$LOG_DIR/runB.log"

if [ -n "${INSTA360_BRIDGE:-}" ]; then
  sleep 15
  timeout 40 "$INSTA360_BRIDGE" --info 2>/dev/null | grep -q found \
    && echo "SHUTDOWN-FIX VALIDATED: camera survived container teardown" \
    || echo "SHUTDOWN-FIX FAILED: camera deaf after container teardown"
fi
sleep 12

if [ -n "${PY_LATENCY_HARNESS:-}" ]; then
  echo "== RUN C: Python full loop"
  ros2 run insta360_camera camera_node > "$LOG_DIR/runC_cam.log" 2>&1 &
  sleep 12
  timeout 150 python3 -u "$PY_LATENCY_HARNESS" 40 2>&1 \
    | grep -E "RESULTS|glass->detect  median|stamp->recv "
  pkill -INT -f "$CAM_BIN"; sleep 6
fi
echo "== ALL RUNS DONE"
