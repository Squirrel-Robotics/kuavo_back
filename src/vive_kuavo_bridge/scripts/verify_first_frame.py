#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_first_frame.py — 验证 vive_arm_bridge 标定后“第一帧零位移”

用法 (上肢 + IK + vive_arm_bridge 都已运行后):
  1) 先启动本脚本:
       rosrun vive_kuavo_bridge verify_first_frame.py
     (或 python3 verify_first_frame.py)
  2) 再调用标定:
       rosservice call /vive_arm_bridge/calibrate "{}"
  3) 进入 ACTIVE 后, 本脚本会在收到首帧 /ik/two_arm_hand_pose_cmd 时,
     对比【命令右手位姿】与【TF 实测 base_link->zarm_r7_end_effector】,
     打印 位置差(米) 与 姿态差(度)。

判定:
  位置差 ≈ 0 且 姿态差 ≈ 0  → 桥接输出零位移正常, 跳变在 IK 侧(种子/控制帧)
  位置差 / 姿态差 明显非零   → 桥接侧就已偏(四元数/帧/读取), 回头查桥接
"""
import math
import numpy as np
import rospy
import tf
from tf.transformations import quaternion_matrix
from kuavo_msgs.msg import twoArmHandPoseCmd

EE_FRAME = rospy.get_param("~ee_frame_right", "zarm_r7_end_effector") if False else "zarm_r7_end_effector"
BASE_FRAME = "base_link"


def quat_angle_deg(q_cmd, q_tf):
    """两个 xyzw 四元数之间的夹角(度)"""
    q_cmd = np.asarray(q_cmd, dtype=np.float64)
    q_tf = np.asarray(q_tf, dtype=np.float64)
    q_cmd /= np.linalg.norm(q_cmd)
    q_tf /= np.linalg.norm(q_tf)
    dot = abs(float(np.dot(q_cmd, q_tf)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


class Verifier:
    def __init__(self):
        rospy.init_node("verify_first_frame", anonymous=True)
        self.listener = tf.TransformListener()
        self.done = False
        rospy.Subscriber("/ik/two_arm_hand_pose_cmd", twoArmHandPoseCmd,
                         self.cb, queue_size=2)
        rospy.loginfo("[verify] 等待首帧 /ik/two_arm_hand_pose_cmd ... 现在去 rosservice call calibrate")

    def cb(self, msg):
        if self.done:
            return
        rp = msg.hand_poses.right_pose
        cmd_pos = np.array(rp.pos_xyz, dtype=np.float64)
        cmd_quat = np.array(rp.quat_xyzw, dtype=np.float64)
        if np.allclose(cmd_pos, 0.0):
            return  # 还没有有效右手目标
        try:
            self.listener.waitForTransform(BASE_FRAME, EE_FRAME,
                                            rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform(BASE_FRAME, EE_FRAME, rospy.Time(0))
        except Exception as e:
            rospy.logwarn("[verify] TF 读取失败, 重试: %s", str(e)[:80])
            return
        tf_pos = np.array(trans, dtype=np.float64)
        tf_quat = np.array(rot, dtype=np.float64)

        d_pos = float(np.linalg.norm(cmd_pos - tf_pos))
        d_ang = quat_angle_deg(cmd_quat, tf_quat)

        print("\n================ 第一帧零位移检查 ================")
        print("frame                : %s -> %s" % (BASE_FRAME, EE_FRAME))
        print("命令 pos  (cmd)      : [% .4f % .4f % .4f]" % tuple(cmd_pos))
        print("实测 pos  (tf)       : [% .4f % .4f % .4f]" % tuple(tf_pos))
        print("命令 quat xyzw       : [% .4f % .4f % .4f % .4f]" % tuple(cmd_quat))
        print("实测 quat xyzw       : [% .4f % .4f % .4f % .4f]" % tuple(tf_quat))
        print("-------------------------------------------------")
        print("位置差 |Δp|          : %.4f m" % d_pos)
        print("姿态差 Δθ            : %.3f deg" % d_ang)
        print("-------------------------------------------------")
        if d_pos < 0.01 and d_ang < 2.0:
            print("结论: ✅ 桥接首帧零位移正常 → 跳变在 IK 侧(种子/控制帧)")
        else:
            print("结论: ⚠️ 桥接命令本身就偏离真实末端 → 问题在桥接(四元数/帧/读取)")
        print("=================================================\n")
        self.done = True
        rospy.signal_shutdown("done")


if __name__ == "__main__":
    Verifier()
    rospy.spin()
