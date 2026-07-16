#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import copy
import os
import sys

import numpy as np
import rosbag
import rospy

from kuavo_msgs.msg import twoArmHandPoseCmd
from kuavo_msgs.srv import fkSrv, twoArmHandPoseCmdSrv


QMEASURED_TOPIC = "/humanoid_wheel/arm_contact_force_debug/qMeasured"
IK_COMMAND_TOPIC = "/ik/two_arm_hand_pose_cmd"


def read_bag_inputs(bag_path):
    q0 = None
    bag_command = None
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, _ in bag.read_messages(
                topics=[QMEASURED_TOPIC, IK_COMMAND_TOPIC]):
            if topic == QMEASURED_TOPIC and q0 is None:
                data = list(msg.data)
                if len(data) < 21:
                    raise RuntimeError(
                        "qMeasured length=%d, expected at least 21" % len(data))
                q0 = np.asarray(data[7:21], dtype=np.float64)
            elif topic == IK_COMMAND_TOPIC and bag_command is None:
                bag_command = copy.deepcopy(msg)
            if q0 is not None and bag_command is not None:
                break
    if q0 is None:
        raise RuntimeError("no %s message in %s" % (QMEASURED_TOPIC, bag_path))
    if bag_command is None:
        raise RuntimeError("no %s message in %s" % (IK_COMMAND_TOPIC, bag_path))
    return q0, bag_command


def read_first_nonzero_delta(csv_path, epsilon):
    with open(csv_path, "r") as csv_file:
        reader = csv.DictReader(csv_file)
        for row_number, row in enumerate(reader, start=1):
            delta = np.asarray([
                float(row["drobot_x"]),
                float(row["drobot_y"]),
                float(row["drobot_z"]),
            ], dtype=np.float64)
            if np.linalg.norm(delta) > epsilon:
                return row_number, delta
    raise RuntimeError("no nonzero robot delta found in %s" % csv_path)


def pose_position(pose):
    return np.asarray(pose.pos_xyz, dtype=np.float64)


def print_vector(label, values, precision=6):
    formatted = np.array2string(
        np.asarray(values), precision=precision, suppress_small=False)
    print("%-24s %s" % (label + ":", formatted))


def main():
    rospy.init_node("offline_first_frame_ik_test", anonymous=False)

    package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    bag_path = os.path.expanduser(rospy.get_param(
        "~bag",
        "~/kuavo-ros-opensource/debug_bags/zero_delta_ik_debug.bag"))
    csv_path = os.path.expanduser(rospy.get_param(
        "~csv",
        os.path.join(package_root, "data", "right_tracker_pose_robot_delta.csv")))
    epsilon = float(rospy.get_param("~delta_epsilon", 1e-9))
    max_joint_delta_deg = float(rospy.get_param("~max_joint_delta_deg", 5.0))
    anchor_source = str(rospy.get_param("~anchor_source", "fk")).lower()
    if anchor_source not in ("fk", "bag_cmd"):
        raise ValueError("anchor_source must be 'fk' or 'bag_cmd'")

    q0, bag_command = read_bag_inputs(bag_path)
    row_number, delta = read_first_nonzero_delta(csv_path, epsilon)

    rospy.wait_for_service("/ik/fk_srv", timeout=10.0)
    rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv", timeout=10.0)
    fk_client = rospy.ServiceProxy("/ik/fk_srv", fkSrv)
    ik_client = rospy.ServiceProxy(
        "/ik/two_arm_hand_pose_cmd_srv", twoArmHandPoseCmdSrv)

    fk_result = fk_client(q0.tolist())
    if not fk_result.success:
        raise RuntimeError("FK service failed for bag q0")

    fk_left_anchor = pose_position(fk_result.hand_poses.left_pose)
    fk_right_anchor = pose_position(fk_result.hand_poses.right_pose)

    if anchor_source == "fk":
        command = twoArmHandPoseCmd()
        command.hand_poses = copy.deepcopy(fk_result.hand_poses)
    else:
        command = copy.deepcopy(bag_command)

    left_anchor = pose_position(command.hand_poses.left_pose)
    right_anchor = pose_position(command.hand_poses.right_pose)
    right_target = right_anchor + delta

    command.hand_poses.right_pose.pos_xyz = right_target.tolist()
    command.hand_poses.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
    command.hand_poses.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
    command.hand_poses.left_pose.joint_angles = q0[:7].tolist()
    command.hand_poses.right_pose.joint_angles = q0[7:14].tolist()
    command.use_custom_ik_param = False
    command.joint_angles_as_q0 = True
    command.frame = 0

    ik_result = ik_client(command)
    if not ik_result.success:
        error_reason = getattr(ik_result, "error_reason", "")
        raise RuntimeError("IK service failed: %s" % error_reason)

    q_solution = np.asarray(ik_result.q_arm, dtype=np.float64)
    if q_solution.size != 14:
        raise RuntimeError(
            "IK returned %d arm joints, expected 14" % q_solution.size)

    delta_q_deg = np.rad2deg(q_solution - q0)
    max_delta = float(np.max(np.abs(delta_q_deg)))

    print("")
    print("=== Offline one-frame IK result (service only; no arm command) ===")
    print("Anchor source:           %s" % anchor_source)
    print("CSV row:                %d" % row_number)
    print_vector("robot delta [m]", delta)
    print_vector("q0 [rad]", q0)
    print_vector("q0 [deg]", np.rad2deg(q0), precision=3)
    print_vector("selected left anchor [m]", left_anchor)
    print_vector("selected right anchor [m]", right_anchor)
    if anchor_source == "bag_cmd":
        print_vector("bag left - FK [m]", left_anchor - fk_left_anchor)
        print_vector("bag right - FK [m]", right_anchor - fk_right_anchor)
    print_vector("right target [m]", right_target)
    print_vector("IK solution [rad]", q_solution)
    print_vector("IK solution [deg]", np.rad2deg(q_solution), precision=3)
    print_vector("solution - q0 [deg]", delta_q_deg, precision=3)
    print("max |solution-q0|:      %.3f deg" % max_delta)

    if max_delta > max_joint_delta_deg:
        print("RESULT: REJECT - exceeds %.3f deg safety threshold" %
              max_joint_delta_deg)
        return 2

    print("RESULT: PASS - within %.3f deg safety threshold" %
          max_joint_delta_deg)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        rospy.logerr("[offline_first_frame_ik_test] %s", exc)
        sys.exit(1)
