#!/usr/bin/env bash
set -eo pipefail

REPO="${HOME}/kuavo-ros-opensource"
LOG="${REPO}/debug_bags/offline_first_frame_ik_node.log"

source "${REPO}/devel/setup.bash"
set -u

roslaunch motion_capture_ik ik_node.launch \
  robot_version:=62 \
  control_hand_side:=2 \
  visualize:=false \
  print_ik_info:=false >"${LOG}" 2>&1 &
IK_LAUNCH_PID=$!

cleanup() {
  if kill -0 "${IK_LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${IK_LAUNCH_PID}" 2>/dev/null || true
    wait "${IK_LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 60); do
  if rosservice info /ik/fk_srv >/dev/null 2>&1 &&
     rosservice info /ik/two_arm_hand_pose_cmd_srv >/dev/null 2>&1; then
    rosrun vive_kuavo_bridge offline_first_frame_ik_test.py "$@"
    exit $?
  fi
  sleep 1
done

echo "IK services did not become ready. See ${LOG}" >&2
exit 1
