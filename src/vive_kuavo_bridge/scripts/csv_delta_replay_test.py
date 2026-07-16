#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_inverse,
    quaternion_multiply,
)


def clamp_vector(v, max_norm):
    n = np.linalg.norm(v)
    if n > max_norm and n > 1e-12:
        return v * (max_norm / n)
    return v


class CsvDeltaReplayTest:
    def __init__(self):
        rospy.init_node("csv_delta_replay_test", anonymous=False)

        self.csv_path = rospy.get_param(
            "~csv_path", "/home/lab/kuavo-ros-opensource/src/vive_kuavo_bridge/data/right_tracker_pose_robot_delta_zero.csv")
        self.output_topic = rospy.get_param(
            "~output_topic", "/vive_arm_bridge/test/replay_delta_base")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))

        # 第一轮建议 0.05，因为你原始轨迹 robot x 最大约 0.45m
        self.scale = float(rospy.get_param("~scale", 0.05))

        # 第一轮最多 2cm，防止后面接机器人时目标太大
        self.max_delta = float(rospy.get_param("~max_delta", 0.02))

        # Optional axis remap applied after reading drobot_x/y/z and before
        # scale/clamp. Examples:
        #   _map_x:=x  _map_y:=y  _map_z:=z   keeps the CSV unchanged.
        #   _map_x:=-y _map_y:=x  _map_z:=z   maps CSV lateral motion to robot forward.
        self.map_x = str(rospy.get_param("~map_x", "x"))
        self.map_y = str(rospy.get_param("~map_y", "y"))
        self.map_z = str(rospy.get_param("~map_z", "z"))

        # Optional relative orientation replay. Disabled by default because
        # position-only tracking is the validated safe path.
        self.enable_orientation = bool(rospy.get_param("~enable_orientation", False))
        self.orientation_scale = float(rospy.get_param("~orientation_scale", 0.25))
        self.max_orientation_delta = math.radians(
            float(rospy.get_param("~max_orientation_delta_deg", 5.0)))
        self.rot_map_roll = str(rospy.get_param("~rot_map_roll", "roll"))
        self.rot_map_pitch = str(rospy.get_param("~rot_map_pitch", "pitch"))
        self.rot_map_yaw = str(rospy.get_param("~rot_map_yaw", "yaw"))

        # 是否循环播放
        self.loop = bool(rospy.get_param("~loop", False))

        self.rows = self.load_csv(self.csv_path)

        self.pub = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=10)

        rospy.loginfo("[csv_delta_replay_test] csv: %s", self.csv_path)
        rospy.loginfo("[csv_delta_replay_test] output: %s", self.output_topic)
        rospy.loginfo("[csv_delta_replay_test] rows: %d", len(self.rows))
        rospy.loginfo("[csv_delta_replay_test] rate=%.1fHz scale=%.3f max_delta=%.3fm loop=%s",
                      self.rate_hz, self.scale, self.max_delta, self.loop)
        rospy.loginfo("[csv_delta_replay_test] axis map: robot_delta=[%s,%s,%s](csv_delta)",
                      self.map_x, self.map_y, self.map_z)
        rospy.loginfo("[csv_delta_replay_test] orientation enable=%s scale=%.3f max=%.2fdeg map=[%s,%s,%s]",
                      self.enable_orientation, self.orientation_scale,
                      math.degrees(self.max_orientation_delta),
                      self.rot_map_roll, self.rot_map_pitch, self.rot_map_yaw)

    def load_csv(self, path):
        rows = []
        q0 = None
        with open(path, "r", newline="") as f:
            reader = csv.DictReader(f)

            required = ["drobot_x", "drobot_y", "drobot_z"]
            quat_fields = [
                "field.pose.orientation.x",
                "field.pose.orientation.y",
                "field.pose.orientation.z",
                "field.pose.orientation.w",
            ]
            if self.enable_orientation:
                required.extend(quat_fields)
            for name in required:
                if name not in reader.fieldnames:
                    raise RuntimeError("CSV 缺少字段: %s" % name)

            for r in reader:
                d = np.array([
                    float(r["drobot_x"]),
                    float(r["drobot_y"]),
                    float(r["drobot_z"]),
                ], dtype=np.float64)
                if self.enable_orientation:
                    q = self.normalize_quat(np.array([
                        float(r[quat_fields[0]]),
                        float(r[quat_fields[1]]),
                        float(r[quat_fields[2]]),
                        float(r[quat_fields[3]]),
                    ], dtype=np.float64))
                    if q0 is None:
                        q0 = q
                    q_delta = self.make_orientation_delta(q0, q)
                else:
                    q_delta = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
                rows.append((d, q_delta))

        if not rows:
            raise RuntimeError("CSV 为空: %s" % path)

        return rows

    def normalize_quat(self, q):
        n = np.linalg.norm(q)
        if n < 1e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        q = q / n
        if q[3] < 0.0:
            q = -q
        return q

    def make_orientation_delta(self, q0, q):
        q_rel = quaternion_multiply(quaternion_inverse(q0), q)
        q_rel = self.normalize_quat(np.asarray(q_rel, dtype=np.float64))
        rpy = np.asarray(euler_from_quaternion(q_rel), dtype=np.float64)
        mapped = np.array([
            self.pick_rotation_axis(rpy, self.rot_map_roll),
            self.pick_rotation_axis(rpy, self.rot_map_pitch),
            self.pick_rotation_axis(rpy, self.rot_map_yaw),
        ], dtype=np.float64)
        mapped *= self.orientation_scale
        norm = np.linalg.norm(mapped)
        if norm > self.max_orientation_delta and norm > 1e-12:
            mapped *= self.max_orientation_delta / norm
        return self.normalize_quat(np.asarray(quaternion_from_euler(
            mapped[0], mapped[1], mapped[2]), dtype=np.float64))

    def remap_delta(self, d):
        return np.array([
            self.pick_axis(d, self.map_x),
            self.pick_axis(d, self.map_y),
            self.pick_axis(d, self.map_z),
        ], dtype=np.float64)

    def pick_axis(self, d, spec):
        spec = spec.strip().lower()
        sign = 1.0
        if spec.startswith("-"):
            sign = -1.0
            spec = spec[1:]
        elif spec.startswith("+"):
            spec = spec[1:]

        if spec == "x":
            return sign * d[0]
        if spec == "y":
            return sign * d[1]
        if spec == "z":
            return sign * d[2]
        if spec in ("0", "zero", "none"):
            return 0.0
        raise RuntimeError("invalid axis map '%s', use x/y/z/-x/-y/-z/0" % spec)

    def pick_rotation_axis(self, rpy, spec):
        spec = spec.strip().lower()
        sign = 1.0
        if spec.startswith("-"):
            sign = -1.0
            spec = spec[1:]
        elif spec.startswith("+"):
            spec = spec[1:]

        if spec in ("roll", "r", "x"):
            return sign * rpy[0]
        if spec in ("pitch", "p", "y"):
            return sign * rpy[1]
        if spec in ("yaw", "z"):
            return sign * rpy[2]
        if spec in ("0", "zero", "none"):
            return 0.0
        raise RuntimeError(
            "invalid rotation map '%s', use roll/pitch/yaw/-roll/-pitch/-yaw/0" % spec)

    def run_once(self):
        rate = rospy.Rate(self.rate_hz)

        for i, row in enumerate(self.rows):
            if rospy.is_shutdown():
                return False

            d_raw, q_delta = row
            d = self.remap_delta(d_raw) * self.scale
            d = clamp_vector(d, self.max_delta)

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link_delta"

            msg.pose.position.x = float(d[0])
            msg.pose.position.y = float(d[1])
            msg.pose.position.z = float(d[2])

            msg.pose.orientation.x = float(q_delta[0])
            msg.pose.orientation.y = float(q_delta[1])
            msg.pose.orientation.z = float(q_delta[2])
            msg.pose.orientation.w = float(q_delta[3])

            self.pub.publish(msg)

            if i % int(max(1, self.rate_hz)) == 0:
                rospy.loginfo("[csv_delta_replay_test] i=%d delta=[%.4f %.4f %.4f]",
                              i, d[0], d[1], d[2])

            rate.sleep()

        rospy.loginfo("[csv_delta_replay_test] replay finished")
        return True

    def run(self):
        rospy.sleep(0.5)

        hold_last = bool(rospy.get_param("~hold_last", True))

        while not rospy.is_shutdown():
            ok = self.run_once()
            if not ok:
                break

            if self.loop:
                continue

            if hold_last:
                rospy.loginfo("[csv_delta_replay_test] holding last delta")
                rate = rospy.Rate(self.rate_hz)
                d_raw, q_delta = self.rows[-1]
                d = self.remap_delta(d_raw) * self.scale
                d = clamp_vector(d, self.max_delta)

                while not rospy.is_shutdown():
                    msg = PoseStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "base_link_delta"
                    msg.pose.position.x = float(d[0])
                    msg.pose.position.y = float(d[1])
                    msg.pose.position.z = float(d[2])
                    msg.pose.orientation.x = float(q_delta[0])
                    msg.pose.orientation.y = float(q_delta[1])
                    msg.pose.orientation.z = float(q_delta[2])
                    msg.pose.orientation.w = float(q_delta[3])
                    self.pub.publish(msg)
                    rate.sleep()

            break


if __name__ == "__main__":
    node = CsvDeltaReplayTest()
    node.run()
