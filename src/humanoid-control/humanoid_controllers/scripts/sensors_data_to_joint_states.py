#!/usr/bin/env python3
"""Publish the Kuavo full-body JointState needed by robot_state_publisher.

This node is state-only. It subscribes to /sensors_data_raw and publishes
sensor_msgs/JointState with the Kuavo v62 20-actuator ordering used by the
controller config. It does not publish commands.
"""

import rospy
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import sensorsData

DEFAULT_JOINT_NAMES = [
    "knee_joint",
    "leg_joint",
    "waist_pitch_joint",
    "waist_yaw_joint",
    "zarm_l1_joint",
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_l5_joint",
    "zarm_l6_joint",
    "zarm_l7_joint",
    "zarm_r1_joint",
    "zarm_r2_joint",
    "zarm_r3_joint",
    "zarm_r4_joint",
    "zarm_r5_joint",
    "zarm_r6_joint",
    "zarm_r7_joint",
    "zhead_1_joint",
    "zhead_2_joint",
]


def _stamp_from_msg(msg):
    stamp = msg.header.stamp
    if stamp and stamp != rospy.Time(0):
        return stamp
    sensor_time = getattr(msg, "sensor_time", 0.0)
    if sensor_time:
        try:
            return rospy.Time.from_sec(float(sensor_time))
        except Exception:
            pass
    return rospy.Time.now()


class SensorsDataToJointStates:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/sensors_data_raw")
        self.output_topic = rospy.get_param("~output_topic", "/joint_states")
        self.joint_names = rospy.get_param("~joint_names", DEFAULT_JOINT_NAMES)
        self.min_len = len(self.joint_names)
        self.warned_short = False
        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, sensorsData, self._callback, queue_size=10)
        rospy.loginfo(
            "sensors_data_to_joint_states: %s -> %s with %d joints",
            self.input_topic,
            self.output_topic,
            self.min_len,
        )

    def _copy_prefix(self, values):
        values = list(values)
        if len(values) >= self.min_len:
            return values[: self.min_len]
        return []

    def _callback(self, msg):
        q = list(msg.joint_data.joint_q)
        if len(q) < self.min_len:
            if not self.warned_short:
                rospy.logwarn(
                    "sensors_data_to_joint_states: joint_q has %d values, need at least %d",
                    len(q),
                    self.min_len,
                )
                self.warned_short = True
            return

        out = JointState()
        out.header.stamp = _stamp_from_msg(msg)
        out.header.frame_id = msg.header.frame_id
        out.name = list(self.joint_names)
        out.position = q[: self.min_len]
        out.velocity = self._copy_prefix(msg.joint_data.joint_v)
        out.effort = self._copy_prefix(msg.joint_data.joint_torque)
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("kuavo_sensors_to_joint_states")
    SensorsDataToJointStates()
    rospy.spin()
