#!/usr/bin/env python3
"""Bridge one Vive tracker yaw delta to Kuavo wheel-arm waist_yaw_joint.

This node is intentionally small and conservative:
  - It subscribes to one PoseStamped tracker topic.
  - It reads the current four low joints from /sensors_data_raw.
  - It publishes /lb_leg_traj with only position[3] changed.
  - It does nothing until /vive_waist_yaw_bridge/calibrate is called.
"""

import math
from typing import List, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from kuavo_msgs.msg import sensorsData
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse


LOW_JOINT_NAMES = [
    "knee_joint",
    "leg_joint",
    "waist_pitch_joint",
    "waist_yaw_joint",
]

HEADING_AXIS_NAMES = ["x", "-x", "y", "-y", "z", "-z"]


def _is_finite_list(values: List[float]) -> bool:
    return all(math.isfinite(float(v)) for v in values)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _shortest_angle_delta(now: float, origin: float) -> float:
    return _wrap_pi(now - origin)


def _axis_vector(name: str) -> List[float]:
    key = str(name).strip().lower()
    sign = -1.0 if key.startswith("-") else 1.0
    axis = key[1:] if key.startswith("-") else key
    if axis == "x":
        return [sign, 0.0, 0.0]
    if axis == "y":
        return [0.0, sign, 0.0]
    if axis == "z":
        return [0.0, 0.0, sign]
    raise ValueError("axis must be one of x/y/z/-x/-y/-z")


def _dot(a: List[float], b: List[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: List[float], b: List[float]) -> List[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _norm(v: List[float]) -> float:
    return math.sqrt(_dot(v, v))


def _normalize(v: List[float]) -> List[float]:
    n = _norm(v)
    if n < 1e-9:
        raise ValueError("zero vector")
    return [v[0] / n, v[1] / n, v[2] / n]


def _quat_to_rot_xyzw(x: float, y: float, z: float, w: float) -> List[List[float]]:
    # Normalize defensively; OpenVR quaternions should already be unit length.
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _mat_vec_mul(m: List[List[float]], v: List[float]) -> List[float]:
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def _euler_rpy_from_quat_xyzw(x: float, y: float, z: float, w: float) -> List[float]:
    # Fixed ROS-style XYZ roll/pitch/yaw, kept for debugging and fallback tests.
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def _heading_from_quat_xyzw(
    x: float,
    y: float,
    z: float,
    w: float,
    world_up_axis: str,
    tracker_forward_axis: str,
) -> float:
    """Return heading around a world up axis using projected tracker forward.

    SteamVR is Y-up, while ROS yaw helpers often assume Z-up. This projection
    avoids roll/pitch twist driving waist yaw when the tracker is held upright.
    """
    up = _normalize(_axis_vector(world_up_axis))
    forward_local = _axis_vector(tracker_forward_axis)
    rot = _quat_to_rot_xyzw(x, y, z, w)
    forward_world = _mat_vec_mul(rot, forward_local)

    # Project the selected tracker forward direction onto the horizontal plane.
    projected = [
        forward_world[0] - up[0] * _dot(forward_world, up),
        forward_world[1] - up[1] * _dot(forward_world, up),
        forward_world[2] - up[2] * _dot(forward_world, up),
    ]
    projected = _normalize(projected)

    # Build a deterministic horizontal basis. Prefer world Z unless it is up.
    reference = [0.0, 0.0, 1.0]
    if abs(_dot(reference, up)) > 0.95:
        reference = [1.0, 0.0, 0.0]
    basis_a = [
        reference[0] - up[0] * _dot(reference, up),
        reference[1] - up[1] * _dot(reference, up),
        reference[2] - up[2] * _dot(reference, up),
    ]
    basis_a = _normalize(basis_a)
    basis_b = _normalize(_cross(up, basis_a))
    return math.atan2(_dot(projected, basis_b), _dot(projected, basis_a))


class ViveWaistYawBridge:
    def __init__(self) -> None:
        self.tracker_topic = rospy.get_param(
            "~tracker_pose_topic", "/vive_arm_bridge/raw/right_tracker_pose")
        self.q_topic = rospy.get_param("~q_topic", "/sensors_data_raw")
        self.output_topic = rospy.get_param("~output_topic", "/lb_leg_traj")
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))

        self.yaw_source = str(rospy.get_param("~yaw_source", "roll")).strip().lower()
        self.world_up_axis = str(rospy.get_param("~world_up_axis", "y")).strip().lower()
        self.tracker_forward_axis = str(
            rospy.get_param("~tracker_forward_axis", "-z")).strip().lower()
        self.yaw_scale = float(rospy.get_param("~yaw_scale", 0.3))
        self.yaw_sign = -1.0 if bool(rospy.get_param("~invert_yaw", False)) else 1.0
        self.max_delta_yaw_deg = abs(float(rospy.get_param("~max_delta_yaw_deg", 5.0)))
        self.max_step_yaw_deg = abs(float(rospy.get_param("~max_step_yaw_deg", 0.5)))
        self.max_tracker_yaw_step_deg = abs(
            float(rospy.get_param("~max_tracker_yaw_step_deg", 20.0)))
        self.tracker_timeout = float(rospy.get_param("~tracker_timeout", 0.5))
        self.q_timeout = float(rospy.get_param("~q_timeout", 1.0))

        self.publish_vr_torso_false = bool(
            rospy.get_param("~publish_vr_torso_false", True))
        self.auto_calibrate = bool(rospy.get_param("~auto_calibrate", False))

        self._latest_low_q_rad: Optional[List[float]] = None
        self._latest_q_time: Optional[rospy.Time] = None
        self._latest_tracker_yaw: Optional[float] = None
        self._latest_tracker_time: Optional[rospy.Time] = None
        self._latest_heading_values = {}
        self._last_raw_tracker_yaw: Optional[float] = None

        self._low0_deg: Optional[List[float]] = None
        self._tracker_yaw0: Optional[float] = None
        self._heading0_values = {}
        self._last_cmd_yaw_deg: Optional[float] = None
        self._active = False

        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=2)
        self.vr_false_pub = rospy.Publisher(
            "/vr_whole_torso_ctrl", Bool, queue_size=1, latch=True)
        self.target_yaw_pub = rospy.Publisher("~debug/target_yaw_deg", Float64, queue_size=2)
        self.delta_yaw_pub = rospy.Publisher("~debug/tracker_delta_yaw_deg", Float64, queue_size=2)
        self.heading_abs_pub = rospy.Publisher(
            "~debug/heading_abs_deg", Float64MultiArray, queue_size=2)
        self.heading_delta_pub = rospy.Publisher(
            "~debug/heading_delta_deg", Float64MultiArray, queue_size=2)

        self.q_sub = rospy.Subscriber(self.q_topic, sensorsData, self._on_q, queue_size=10)
        self.tracker_sub = rospy.Subscriber(
            self.tracker_topic, PoseStamped, self._on_tracker, queue_size=10)
        self.calib_srv = rospy.Service(
            "~calibrate", Trigger, self._handle_calibrate)
        self.stop_srv = rospy.Service(
            "~stop", Trigger, self._handle_stop)

        rospy.loginfo(
            "[vive_waist_yaw_bridge] tracker=%s q=%s output=%s rate=%.1fHz",
            self.tracker_topic, self.q_topic, self.output_topic, self.publish_rate)
        rospy.loginfo(
            "[vive_waist_yaw_bridge] yaw_source=%s world_up=%s tracker_forward=%s",
            self.yaw_source, self.world_up_axis, self.tracker_forward_axis)
        rospy.loginfo(
            "[vive_waist_yaw_bridge] heading debug order: %s",
            ", ".join(HEADING_AXIS_NAMES))
        rospy.loginfo(
            "[vive_waist_yaw_bridge] yaw_scale=%.3f invert=%s max_delta=%.2fdeg max_step=%.2fdeg",
            self.yaw_scale, self.yaw_sign < 0.0, self.max_delta_yaw_deg, self.max_step_yaw_deg)
        rospy.loginfo(
            "[vive_waist_yaw_bridge] call: rosservice call /vive_waist_yaw_bridge/calibrate")

    def _on_q(self, msg: sensorsData) -> None:
        q = list(msg.joint_data.joint_q[:4])
        if len(q) != 4 or not _is_finite_list(q):
            rospy.logwarn_throttle(
                1.0, "[vive_waist_yaw_bridge] invalid low joint q from %s", self.q_topic)
            return
        self._latest_low_q_rad = [float(v) for v in q]
        self._latest_q_time = rospy.Time.now()

    def _on_tracker(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        if not _is_finite_list([q.x, q.y, q.z, q.w]):
            rospy.logwarn_throttle(1.0, "[vive_waist_yaw_bridge] invalid tracker quat")
            return
        try:
            heading_values = self._compute_heading_values(q.x, q.y, q.z, q.w)
            self._latest_heading_values = dict(heading_values)
            self._publish_heading_debug(heading_values)

            if self.yaw_source == "world_yaw":
                yaw = _heading_from_quat_xyzw(
                    q.x, q.y, q.z, q.w, self.world_up_axis, self.tracker_forward_axis)
            elif self.yaw_source.startswith("heading_"):
                axis = self.yaw_source[len("heading_"):]
                axis = axis.replace("neg_", "-").replace("minus_", "-")
                if axis not in heading_values:
                    rospy.logwarn_throttle(
                        1.0,
                        "[vive_waist_yaw_bridge] unsupported heading axis '%s'; use heading_x/heading_-x/.../heading_-z",
                        axis,
                    )
                    return
                yaw = heading_values[axis]
            else:
                roll, pitch, yaw_z = _euler_rpy_from_quat_xyzw(q.x, q.y, q.z, q.w)
                if self.yaw_source == "roll":
                    yaw = roll
                elif self.yaw_source == "pitch":
                    yaw = pitch
                elif self.yaw_source in ("yaw", "yaw_z", "euler_yaw"):
                    yaw = yaw_z
                else:
                    rospy.logwarn_throttle(
                        1.0,
                        "[vive_waist_yaw_bridge] unsupported yaw_source '%s'; use world_yaw/roll/pitch/yaw",
                        self.yaw_source,
                    )
                    return
        except Exception as e:
            rospy.logwarn_throttle(
                1.0, "[vive_waist_yaw_bridge] failed to compute tracker yaw: %s", str(e)[:100])
            return
        if self._last_raw_tracker_yaw is not None and self.max_tracker_yaw_step_deg > 0.0:
            step_deg = abs(math.degrees(_shortest_angle_delta(yaw, self._last_raw_tracker_yaw)))
            if step_deg > self.max_tracker_yaw_step_deg:
                rospy.logwarn_throttle(
                    1.0,
                    "[vive_waist_yaw_bridge] tracker yaw jump rejected: %.2fdeg > %.2fdeg",
                    step_deg,
                    self.max_tracker_yaw_step_deg,
                )
                return
        self._last_raw_tracker_yaw = yaw
        self._latest_tracker_yaw = yaw
        self._latest_tracker_time = rospy.Time.now()

    def _compute_heading_values(self, x: float, y: float, z: float, w: float):
        values = {}
        for axis in HEADING_AXIS_NAMES:
            try:
                values[axis] = _heading_from_quat_xyzw(
                    x, y, z, w, self.world_up_axis, axis)
            except Exception:
                values[axis] = float("nan")
        return values

    def _publish_heading_debug(self, values) -> None:
        abs_msg = Float64MultiArray()
        abs_msg.data = [
            math.degrees(values.get(axis, float("nan")))
            for axis in HEADING_AXIS_NAMES
        ]
        self.heading_abs_pub.publish(abs_msg)

        delta_msg = Float64MultiArray()
        if self._heading0_values:
            delta_msg.data = [
                math.degrees(_shortest_angle_delta(
                    values.get(axis, float("nan")),
                    self._heading0_values.get(axis, float("nan"))))
                if (
                    math.isfinite(values.get(axis, float("nan")))
                    and math.isfinite(self._heading0_values.get(axis, float("nan")))
                )
                else float("nan")
                for axis in HEADING_AXIS_NAMES
            ]
        else:
            delta_msg.data = [float("nan")] * len(HEADING_AXIS_NAMES)
        self.heading_delta_pub.publish(delta_msg)

    def _fresh(self, stamp: Optional[rospy.Time], timeout: float) -> bool:
        if stamp is None:
            return False
        return (rospy.Time.now() - stamp).to_sec() <= timeout

    def _calibrate(self) -> TriggerResponse:
        if self._latest_low_q_rad is None or not self._fresh(self._latest_q_time, self.q_timeout):
            return TriggerResponse(False, "no fresh /sensors_data_raw low joint q")
        if self._latest_tracker_yaw is None or not self._fresh(
                self._latest_tracker_time, self.tracker_timeout):
            return TriggerResponse(False, "no fresh tracker pose")

        self._low0_deg = [math.degrees(v) for v in self._latest_low_q_rad]
        self._tracker_yaw0 = self._latest_tracker_yaw
        self._heading0_values = dict(self._latest_heading_values)
        self._last_cmd_yaw_deg = self._low0_deg[3]
        self._active = True
        self._publish_vr_false()
        self._publish_target(self._low0_deg[3], 0.0)

        msg = (
            "calibrated low0_deg=[%.3f, %.3f, %.3f, %.3f], tracker_yaw0=%.2fdeg"
            % (
                self._low0_deg[0],
                self._low0_deg[1],
                self._low0_deg[2],
                self._low0_deg[3],
                math.degrees(self._tracker_yaw0),
            )
        )
        rospy.loginfo("[vive_waist_yaw_bridge] %s", msg)
        return TriggerResponse(True, msg)

    def _handle_calibrate(self, _req) -> TriggerResponse:
        return self._calibrate()

    def _handle_stop(self, _req) -> TriggerResponse:
        self._active = False
        if self._low0_deg is not None:
            self._publish_target(self._low0_deg[3], 0.0)
        return TriggerResponse(True, "stopped; published calibrated hold position")

    def _publish_vr_false(self) -> None:
        if self.publish_vr_torso_false:
            self.vr_false_pub.publish(Bool(data=False))

    def _publish_target(self, waist_yaw_deg: float, tracker_delta_deg: float) -> None:
        if self._low0_deg is None:
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(LOW_JOINT_NAMES)
        msg.position = list(self._low0_deg)
        msg.position[3] = waist_yaw_deg
        msg.velocity = [0.0, 0.0, 0.0, 0.0]
        msg.effort = []
        self.pub.publish(msg)
        self.target_yaw_pub.publish(Float64(data=waist_yaw_deg))
        self.delta_yaw_pub.publish(Float64(data=tracker_delta_deg))

    def _tick(self) -> None:
        self._publish_vr_false()

        if self.auto_calibrate and not self._active:
            resp = self._calibrate()
            if not resp.success:
                rospy.logwarn_throttle(2.0, "[vive_waist_yaw_bridge] auto_calibrate: %s", resp.message)
            return

        if not self._active:
            return
        if self._low0_deg is None or self._tracker_yaw0 is None:
            return
        if self._latest_tracker_yaw is None or not self._fresh(
                self._latest_tracker_time, self.tracker_timeout):
            rospy.logwarn_throttle(
                1.0, "[vive_waist_yaw_bridge] tracker timeout; stop publishing")
            return

        raw_delta_deg = math.degrees(
            _shortest_angle_delta(self._latest_tracker_yaw, self._tracker_yaw0))
        scaled_delta_deg = self.yaw_sign * self.yaw_scale * raw_delta_deg
        limited_delta_deg = _clamp(
            scaled_delta_deg, -self.max_delta_yaw_deg, self.max_delta_yaw_deg)
        target_yaw_deg = self._low0_deg[3] + limited_delta_deg

        if self._last_cmd_yaw_deg is not None and self.max_step_yaw_deg > 0.0:
            step = _clamp(
                target_yaw_deg - self._last_cmd_yaw_deg,
                -self.max_step_yaw_deg,
                self.max_step_yaw_deg,
            )
            target_yaw_deg = self._last_cmd_yaw_deg + step

        self._last_cmd_yaw_deg = target_yaw_deg
        self._publish_target(target_yaw_deg, limited_delta_deg)

    def spin(self) -> None:
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()


def main() -> None:
    rospy.init_node("vive_waist_yaw_bridge")
    ViveWaistYawBridge().spin()


if __name__ == "__main__":
    main()
