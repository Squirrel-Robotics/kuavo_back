#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from kuavo_msgs.srv import fkSrv
from tf.transformations import quaternion_multiply

from replay_ee_pose_to_ik_cmd_preview import ArmQ0Provider


class ReplayDeltaToEePoseTest:
    def __init__(self):
        rospy.init_node("replay_delta_to_ee_pose_test", anonymous=False)

        self.input_topic = rospy.get_param(
            "~input_topic", "/vive_arm_bridge/test/replay_delta_base")
        self.output_topic = rospy.get_param(
            "~output_topic", "/vive_arm_bridge/test/replay_right_ee_pose")

        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.ee_frame = rospy.get_param("~ee_frame", "zarm_r7_end_effector")

        self.fk_service_name = rospy.get_param("~fk_service", "/ik/fk_srv")
        self.q0_wait_timeout = float(rospy.get_param("~q0_wait_timeout", 10.0))
        self.enable_orientation = bool(rospy.get_param("~enable_orientation", False))
        self.orientation_apply_order = str(rospy.get_param(
            "~orientation_apply_order", "local")).lower()
        self.q0_provider = ArmQ0Provider("replay_delta_to_ee_pose_test")

        self.ee0_pos = None
        self.ee0_quat = None

        self.pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, PoseStamped, self.cb, queue_size=10)

        rospy.loginfo("[replay_delta_to_ee_pose_test] input : %s", self.input_topic)
        rospy.loginfo("[replay_delta_to_ee_pose_test] output: %s", self.output_topic)
        rospy.loginfo("[replay_delta_to_ee_pose_test] FK service: %s", self.fk_service_name)
        rospy.loginfo("[replay_delta_to_ee_pose_test] orientation enable=%s order=%s",
                      self.enable_orientation, self.orientation_apply_order)

        self.wait_and_record_ee0()

    def wait_and_record_ee0(self):
        rospy.loginfo("[replay_delta_to_ee_pose_test] waiting for q0 and FK ...")

        if not self.q0_provider.wait_for_q0(self.q0_wait_timeout):
            raise RuntimeError("no valid q0 from %s" % self.q0_provider.topic)
        rospy.wait_for_service(self.fk_service_name, timeout=10.0)
        fk_client = rospy.ServiceProxy(self.fk_service_name, fkSrv)
        result = fk_client(self.q0_provider.q_arm().tolist())
        if not result.success:
            raise RuntimeError("%s failed for current q0" % self.fk_service_name)

        right_pose = result.hand_poses.right_pose
        self.ee0_pos = np.array(right_pose.pos_xyz, dtype=np.float64)
        self.ee0_quat = np.array(right_pose.quat_xyzw, dtype=np.float64)  # xyzw

        rospy.loginfo(
            "[replay_delta_to_ee_pose_test] FK ee0 pos = [%.4f %.4f %.4f]",
            self.ee0_pos[0], self.ee0_pos[1], self.ee0_pos[2]
        )
        rospy.loginfo(
            "[replay_delta_to_ee_pose_test] ee0 quat_xyzw = [%.4f %.4f %.4f %.4f]",
            self.ee0_quat[0], self.ee0_quat[1], self.ee0_quat[2], self.ee0_quat[3]
        )

    def cb(self, msg):
        if self.ee0_pos is None or self.ee0_quat is None:
            return

        delta = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=np.float64)

        target = self.ee0_pos + delta
        q_delta = self.normalize_quat(np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ], dtype=np.float64))
        if self.enable_orientation:
            if self.orientation_apply_order == "base":
                target_quat = quaternion_multiply(q_delta, self.ee0_quat)
            else:
                target_quat = quaternion_multiply(self.ee0_quat, q_delta)
            target_quat = self.normalize_quat(np.asarray(target_quat, dtype=np.float64))
        else:
            target_quat = self.ee0_quat

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame

        out.pose.position.x = float(target[0])
        out.pose.position.y = float(target[1])
        out.pose.position.z = float(target[2])

        out.pose.orientation.x = float(target_quat[0])
        out.pose.orientation.y = float(target_quat[1])
        out.pose.orientation.z = float(target_quat[2])
        out.pose.orientation.w = float(target_quat[3])

        self.pub.publish(out)

    def normalize_quat(self, q):
        n = np.linalg.norm(q)
        if n < 1e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        q = q / n
        if q[3] < 0.0:
            q = -q
        return q


if __name__ == "__main__":
    node = ReplayDeltaToEePoseTest()
    rospy.spin()
