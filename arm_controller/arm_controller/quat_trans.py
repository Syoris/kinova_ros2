import math
import numpy as np

import rclpy
import rclpy.action
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return np.array(q)


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class QuatTrans(Node):
    def __init__(self):
        super().__init__("quat_transformer")
        self.subscription = self.create_subscription(
            PoseStamped, "/arm/out/tool_pose", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        msg_data = msg.pose

        quaternion = (
            msg_data.orientation.x,
            msg_data.orientation.y,
            msg_data.orientation.z,
            msg_data.orientation.w,
        )
        r, p, y = euler_from_quaternion(quaternion)

        self.get_logger().info("Roll: {0}, Pitch: {1}, Yaw: {2}".format(r, p, y))


def main():
    rclpy.init()

    quat_trans = QuatTrans()

    rclpy.spin(quat_trans)


if __name__ == "__main__":
    main()
