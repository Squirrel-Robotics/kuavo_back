#!/usr/bin/env bash
set -euo pipefail

cd "${KUAVO_WS:-$HOME/kuavo-ros-opensource}"
source devel/setup.bash

DURATION="${1:-60}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${2:-$PWD/debug_bags}"
mkdir -p "$OUT_DIR"

OUT_BAG="$OUT_DIR/elbow_ik_debug_${STAMP}.bag"

TOPICS=(
  /rosout
  /rosout_agg

  # Vive tracker raw input
  /vive_arm_bridge/raw/right_tracker_pose
  /vive_arm_bridge/raw/left_tracker_pose

  # Bridge-computed target EE pose for visualization
  /vive_arm_bridge/viz/right_hand
  /vive_arm_bridge/viz/left_hand

  # IK input: desired two-arm hand poses, q0, elbow hint
  /ik/two_arm_hand_pose_cmd

  # IK output: solved arm joint trajectory
  /kuavo_arm_traj

  # Final command to robot / lower control
  /joint_cmd
  /lb_joint_cmd
  /humanoid_wheel/target_qpos

  # Current robot feedback
  /sensors_data_raw
  /humanoid_wheel/eePoses
  /humanoid_wheel/arm_contact_force_debug/qMeasured
  /humanoid_wheel/ik_lb_low_Joint
  /humanoid_wheel/filter_lb_low_Joint

  # Controller measured / optimized states
  /humanoid_controller/measuredRbdState_/joint_pos
  /humanoid_controller/measuredRbdState_/joint_vel
  /humanoid_controller/optimizedState_mrt_/joint_pos

  # Joint state aliases
  /joint_states
  /omni_bot/joint_states

  # TF: reconstruct elbow/end-effector actual motion
  /tf
  /tf_static
)

echo "[record_elbow_ik_debug_bag] output   : $OUT_BAG"
echo "[record_elbow_ik_debug_bag] duration : ${DURATION}s"
echo "[record_elbow_ik_debug_bag] topics   : ${#TOPICS[@]}"
echo "[record_elbow_ik_debug_bag] start recording ..."

rosbag record \
  --lz4 \
  --buffsize=4096 \
  --duration="${DURATION}" \
  -O "$OUT_BAG" \
  "${TOPICS[@]}"

echo "[record_elbow_ik_debug_bag] done: $OUT_BAG"