#!/usr/bin/env bash
set -euo pipefail

cd "${KUAVO_WS:-$HOME/kuavo-ros-opensource}"
source devel/setup.bash

DURATION="${1:-60}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${2:-$PWD/debug_bags}"
mkdir -p "$OUT_DIR"

OUT_BAG="$OUT_DIR/vive_debug_${STAMP}.bag"

TOPICS=(
  /rosout
  /rosout_agg

  /vive_arm_bridge/raw/right_tracker_pose
  /vive_arm_bridge/raw/left_tracker_pose
  /vive_arm_bridge/test/replay_delta_base
  /vive_arm_bridge/test/replay_right_ee_pose
  /vive_arm_bridge/test/replay_ik_cmd_preview

  /ik/two_arm_hand_pose_cmd
  /kuavo_arm_traj
  /joint_cmd
  /lb_joint_cmd

  /sensors_data_raw
  /humanoid_wheel/eePoses
  /humanoid_wheel/target_qpos
  /humanoid_wheel/arm_contact_force_debug/qMeasured
  /humanoid_wheel/ik_lb_low_Joint
  /humanoid_wheel/filter_lb_low_Joint

  /humanoid_controller/measuredRbdState_/joint_pos
  /humanoid_controller/measuredRbdState_/joint_vel
  /humanoid_controller/optimizedState_mrt_/joint_pos

  /joint_states
  /omni_bot/joint_states

  /tf
  /tf_static
)

echo "[record_vive_debug_bag] output   : $OUT_BAG"
echo "[record_vive_debug_bag] duration : ${DURATION}s"
echo "[record_vive_debug_bag] topics   : ${#TOPICS[@]}"
echo "[record_vive_debug_bag] start rosbag record ..."

rosbag record \
  --lz4 \
  --buffsize=2048 \
  --duration="${DURATION}" \
  -O "$OUT_BAG" \
  "${TOPICS[@]}"

echo "[record_vive_debug_bag] done: $OUT_BAG"
