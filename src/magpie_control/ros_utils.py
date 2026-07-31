"""
ros_utils.py — conversions between ROS 2 geometry messages and the pose
representations used across magpie_control (axis-angle 6-vectors and 4x4
homogeneous transforms).

Consolidated here so ur5_node and deligrasp_node share one implementation
instead of each carrying a private copy (PR #2 review). The rotation math is
kept explicit to preserve the exact hardware-verified conventions; it could be
swapped for scipy.spatial.transform.Rotation in a follow-up without changing
these call signatures.
"""

import numpy as np
from geometry_msgs.msg import Pose

from magpie_control import poses


def axisangle_to_quat(rv):
    """Axis-angle rotation vector -> (w, x, y, z) quaternion."""
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    axis = rv / angle
    s = np.sin(angle / 2.0)
    return (np.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_to_axisangle(w, x, y, z):
    """(w, x, y, z) quaternion -> axis-angle rotation vector."""
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sin(angle / 2.0)
    if s < 1e-10:
        return np.zeros(3)
    return angle * np.array([x, y, z]) / s


def pose_vec_to_msg(vec):
    """6-element [x, y, z, rx, ry, rz] axis-angle pose vector -> geometry_msgs/Pose.

    Use when you already hold the raw RTDE 6-vector (e.g.
    ``ur5.recv.getActualTCPPose()``) to avoid a needless round-trip through a
    4x4 matrix.
    """
    w, x, y, z = axisangle_to_quat(np.array(vec[3:6]))
    msg = Pose()
    msg.position.x = vec[0]
    msg.position.y = vec[1]
    msg.position.z = vec[2]
    msg.orientation.w = w
    msg.orientation.x = x
    msg.orientation.y = y
    msg.orientation.z = z
    return msg


def pose_msg_to_matrix(pose):
    """geometry_msgs/Pose -> 4x4 homogeneous matrix."""
    rv = quat_to_axisangle(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    )
    vec = [pose.position.x, pose.position.y, pose.position.z,
           rv[0], rv[1], rv[2]]
    return poses.pose_vec_to_mtrx(vec)


def matrix_to_pose_msg(matrix):
    """4x4 homogeneous matrix -> geometry_msgs/Pose."""
    vec = poses.pose_mtrx_to_vec(np.array(matrix))
    return pose_vec_to_msg(vec)
