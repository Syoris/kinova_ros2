from rclpy.node import Node

from kinova_msgs.msg import JointAngles, JointVelocity


class KinovaSim(Node):
    def __init__(self):
        super().__init__("kinova_sim")
        self.get_logger().info("KinovaSim initialization...")

        self.joint_angles = {
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
            "joint5": 0.0,
            "joint6": 0.0,
            "joint7": 0.0,
        }

        self.joint_speeds = {
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
            "joint5": 0.0,
            "joint6": 0.0,
            "joint7": 0.0,
        }

        self.joint_angles_pub_ = self.create_publisher(JointAngles, "/arm/out/joint_angles", 5)

        # Joint vels
        self._joint_vels_sub = self.create_subscription(
            JointVelocity, "/arm/in/joint_velocity", self._joint_vels_callback, 10
        )
        self._joint_vels_sub

        self.pub_rate = 5
        self.pub_timer = self.create_timer(1 / self.pub_rate, self._timer_callback)

        self.get_logger().info("KinovaSim initialization done")

    def _timer_callback(self):
        # self.get_logger().info("Timer callback")

        msg = JointAngles()
        for (joint, angle), speed in zip(self.joint_angles.items(), self.joint_speeds.values()):
            angle += speed / self.pub_rate
            self.joint_angles[joint] = angle
            self.joint_speeds[joint] = 0
            setattr(msg, joint, angle)

        self.joint_angles_pub_.publish(msg)

    def _joint_vels_callback(self, msg):
        self.joint_speeds["joint1"] = msg.joint1
        self.joint_speeds["joint2"] = msg.joint2
        self.joint_speeds["joint3"] = msg.joint3
        self.joint_speeds["joint4"] = msg.joint4
        self.joint_speeds["joint5"] = msg.joint5
        self.joint_speeds["joint6"] = msg.joint6
        self.joint_speeds["joint7"] = msg.joint7
        # self.get_logger().info(f"Received joint velocities: {self.joint_speeds}")
