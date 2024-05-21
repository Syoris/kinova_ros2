import math
import numpy as np

import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions

from action_msgs.msg import GoalStatus
from kinova_msgs.msg import JointAngles, JointVelocity, PoseVelocity
from kinova_msgs.srv import (
    Start,
    Stop,
    HomeArm,
    AddPoseToCartesianTrajectory,
    ClearTrajectories,
    ZeroTorques,
    RunCOMParametersEstimation,
)
from kinova_msgs.action import ArmJointAngles, ArmPose

from geometry_msgs.msg import WrenchStamped

from arm_controller.ActionClients import GoToAnglesClient, GoToPoseClient
from enum import Enum
import time


def quaternion_from_euler(rpy, degrees=False):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = rpy[0]
    y = rpy[1]
    z = rpy[2]

    if degrees:
        x = math.radians(x)
        y = math.radians(y)
        z = math.radians(z)

    cx = math.cos(x * 0.5)
    sx = math.sin(x * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cz = math.cos(z * 0.5)
    sz = math.sin(z * 0.5)

    q = [0] * 4
    q[0] = sx * cy * cz + cx * sy * sz
    q[1] = -sx * cy * sz + cx * sy * cz
    q[2] = sx * sy * cz + cx * cy * sz
    q[3] = -sx * sy * sz + cx * cy * cz

    # # Doesnt seem to be the correct convention
    # roll = rpy[0]
    # pitch = rpy[1]
    # yaw = rpy[2]

    # cy = math.cos(yaw * 0.5)
    # sy = math.sin(yaw * 0.5)
    # cp = math.cos(pitch * 0.5)
    # sp = math.sin(pitch * 0.5)
    # cr = math.cos(roll * 0.5)
    # sr = math.sin(roll * 0.5)

    # q = [0] * 4
    # q[0] = cy * cp * cr + sy * sp * sr
    # q[1] = cy * cp * sr - sy * sp * cr
    # q[2] = sy * cp * sr + cy * sp * cr
    # q[3] = sy * cp * cr - cy * sp * sr

    return np.array(q)


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ControlMode(Enum):
    REST_MODE = 1
    JOINT_MODE = 2
    POSE_MODE = 3


class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")

        self._is_homed = False
        self._control_mode = self.set_control_mode(ControlMode.REST_MODE)

        self.target_joint_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_pose_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._init_clients()
        self._init_publishers()
        self._init_action_clients()

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("ArmController node has been initialized.")

    def set_control_mode(self, mode: ControlMode):
        self.get_logger().info(f"Setting control mode to {mode}")
        self._control_mode = mode

    def cancel_actions(self):
        self.get_logger().info("Cancelling all actions...")

        self._go_to_angles_client.cancel_goal()
        self._go_to_pose_client.cancel_goal()

    # ---------------- SERVICE CLIENTS ----------------
    # MARK: SERVICES
    def _init_clients(self):
        # ------ Clients ------
        self.get_logger().info("Creating clients...")

        # Start client
        self.start_cli = self.create_client(Start, "/arm/in/start")
        while not self.start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("start service not available, waiting again...")
        self.start_cli_req = Start.Request()

        # Stop client
        self.stop_cli = self.create_client(Stop, "/arm/in/stop")
        while not self.stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("stop service not available, waiting again...")
        self.stop_cli_req = Stop.Request()

        # Home arm
        self.home_arm_cli = self.create_client(HomeArm, "/arm/in/home_arm")
        while not self.home_arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("home_arm service not available, waiting again...")
        self.home_arm_cli_req = HomeArm.Request()

        # Add pose to cartesian trajectory
        self.add_pose_to_cartesian_trajectory_cli = self.create_client(
            AddPoseToCartesianTrajectory, "/arm/in/add_pose_to_Cartesian_trajectory"
        )
        while not self.add_pose_to_cartesian_trajectory_cli.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "add_pose_to_cartesian_trajectory service not available, waiting again..."
            )
        self.add_pose_to_cartesian_trajectory_cli_req = (
            AddPoseToCartesianTrajectory.Request()
        )

        # Clear trajectories
        self.clear_trajectories_cli = self.create_client(
            ClearTrajectories, "/arm/in/clear_trajectories"
        )
        while not self.clear_trajectories_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "clear_trajectories service not available, waiting again..."
            )
        self.clear_trajectories_cli_req = ClearTrajectories.Request()

        # Zero torques
        self.zero_torques_cli = self.create_client(
            ZeroTorques, "/arm/in/set_zero_torques"
        )
        while not self.zero_torques_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "zero_torques service not available, waiting again..."
            )
        self.zero_torques_cli_req = ZeroTorques.Request()

        # Run COM parameters estimation
        self.run_com_parameters_estimation_cli = self.create_client(
            RunCOMParametersEstimation, "/arm/in/run_COM_parameters_estimation"
        )
        while not self.run_com_parameters_estimation_cli.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "run_COM_parameters_estimation service not available, waiting again..."
            )
        self.run_com_parameters_estimation_cli_req = (
            RunCOMParametersEstimation.Request()
        )

    def start_arm(self):
        self.get_logger().info("Starting the arm...")

        self.future = self.start_cli.call_async(self.start_cli_req)
        rclpy.spin_until_future_complete(self, self.future)

        res = self.future.result()
        self.get_logger().info(f"Result: {res.start_result}")
        return res

    def stop_arm(self):
        self.get_logger().info("Stopping the arm...")

        self.future = self.stop_cli.call_async(self.stop_cli_req)
        rclpy.spin_until_future_complete(self, self.future)

        res = self.future.result()
        self.get_logger().info(f"Result: {res.stop_result}")

        return res

    def home_arm(self):
        self.get_logger().info("Homing the arm...")

        self.future = self.home_arm_cli.call_async(self.home_arm_cli_req)
        rclpy.spin_until_future_complete(self, self.future)

        res = self.future.result()

        self.get_logger().info(f"Result: {res.homearm_result}")
        self._is_homed = True

        return res

    def run_com_parameters_estimation(self):
        self.get_logger().info("Running COM parameters estimation...")

        self.future = self.run_com_parameters_estimation_cli.call_async(
            self.run_com_parameters_estimation_cli_req
        )
        rclpy.spin_until_future_complete(self, self.future)

        res = self.future.result()
        self.get_logger().info(f"Result: {res.run_COM_parameters_estimation_result}")

        return res

    def zero_torques(self):
        self.get_logger().info("Zeroing torques...")

        self.future = self.zero_torques_cli.call_async(self.zero_torques_cli_req)
        rclpy.spin_until_future_complete(self, self.future)

        res = self.future.result()
        self.get_logger().info(f"Result: {res.result}")

        return res

    def clear_trajectories(self):
        self.get_logger().info("Clearing trajectories...")

        self.future = self.clear_trajectories_cli.call_async(
            self.clear_trajectories_cli_req
        )
        rclpy.spin_until_future_complete(self, self.future)

        res = self.future.result()
        self.get_logger().info(f"Result: {res.result}")

        return res

    # ---------------- PUBLISHERS ----------------
    # MARK: PUBLISHERS
    def _init_publishers(self):
        self.get_logger().info("Creating publishers...")

        # Joint vels publisher
        self.joint_vels_pub_ = self.create_publisher(
            JointVelocity, "/arm/in/joint_velocity", 5
        )

        # Cartesian pose publisher
        self.cartesian_pose_vels_pub_ = self.create_publisher(
            PoseVelocity, "/arm/in/cartesian_velocity", 5
        )

    def publish_joint_vels(self):
        joint_vels = self.target_joint_vels

        msg = JointVelocity()
        msg.joint1 = joint_vels[0]
        msg.joint2 = joint_vels[1]
        msg.joint3 = joint_vels[2]
        msg.joint4 = joint_vels[3]
        msg.joint5 = joint_vels[4]
        msg.joint6 = joint_vels[5]
        msg.joint7 = joint_vels[6]

        self.joint_vels_pub_.publish(msg)

    def publish_pose_vels(self):
        pose_vels = self.target_pose_vels

        msg = PoseVelocity()

        msg.twist_linear_x = pose_vels[0]
        msg.twist_linear_y = pose_vels[1]
        msg.twist_linear_z = pose_vels[2]
        msg.twist_angular_x = pose_vels[3]
        msg.twist_angular_y = pose_vels[4]
        msg.twist_angular_z = pose_vels[5]

        self.cartesian_pose_vels_pub_.publish(msg)

    def timer_callback(self):
        if not self._is_homed:
            return

        # self.get_logger().info("Timer callback")
        if self._control_mode == ControlMode.JOINT_MODE:
            self.publish_joint_vels()

        elif self._control_mode == ControlMode.POSE_MODE:
            self.publish_pose_vels()

    # ---------------- ACTION CLIENTS ----------------
    # MARK: ACTIONS
    def _init_action_clients(self):
        self.get_logger().info("Creating action clients...")

        self._go_to_angles_client = GoToAnglesClient(
            self, action_type=ArmJointAngles, action_name="/arm/go_to_angles"
        )

        self._go_to_pose_client = GoToPoseClient(
            self, action_type=ArmPose, action_name="/arm/go_to_pose"
        )

    def go_to_angles(self, angles: list):
        """Action

        Args:
            angles (list): Target angle of each joint
        """
        goal_msg = ArmJointAngles.Goal()
        goal_msg.angles.joint1 = float(angles[0])
        goal_msg.angles.joint2 = float(angles[1])
        goal_msg.angles.joint3 = float(angles[2])
        goal_msg.angles.joint4 = float(angles[3])
        goal_msg.angles.joint5 = float(angles[4])
        goal_msg.angles.joint6 = float(angles[5])
        goal_msg.angles.joint7 = float(angles[6])

        self._go_to_angles_client.send_goal(goal_msg)

    def go_to_pose(self, pose: list, orientation: list):
        goal_msg = ArmPose.Goal()

        self.get_logger().info(f"Going to pose: {pose}\n{orientation}")

        goal_msg.pose.pose.position.x = float(pose[0])
        goal_msg.pose.pose.position.y = float(pose[1])
        goal_msg.pose.pose.position.z = float(pose[2])

        goal_msg.pose.pose.orientation.x = float(orientation[0])
        goal_msg.pose.pose.orientation.y = float(orientation[1])
        goal_msg.pose.pose.orientation.z = float(orientation[2])
        goal_msg.pose.pose.orientation.w = float(orientation[3])

        self._go_to_pose_client.send_goal(goal_msg)

    def go_angles(self, angles: list):
        self.get_logger().info("Going to angles...")


def main():
    # rclpy.init()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    try:
        arm_controller = ArmController()

        # Start the arm
        arm_controller.start_arm()

        # Home the arm
        arm_controller.home_arm()
        time.sleep(1)

        # # Pusblish joint vels
        # joint_vels = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 5.0]
        # arm_controller.target_joint_vels = joint_vels
        # arm_controller.set_control_mode(ControlMode.JOINT_MODE)

        # # Go to angles
        # # home_angles = [103.0, 197, 180, 43, 265, 257, 288]
        # angles = [180.0, 180.0, 180.0, 180, 180.0, 180.0, 180.0]
        # arm_controller.go_to_angles(angles)
        # time.sleep(2)

        # Go to pose
        home_position = [0.20, -0.26, 0.5]
        home_orientation = np.array(
            [
                -0.14027640223503113,
                0.7090284824371338,
                -0.42349401116371155,
                0.5461264252662659,
            ]
        )
        position_d = [0, 0, 0]
        orientation_d = [0, 0, 0]

        # position_goal = [x + y for x, y in zip(home_position, position_d)]
        # orientation_goal = home_orientation + quaternion_from_euler(orientation_d)

        position_goal = [0.4, -0.2, 0.3]

        orientation_goal = quaternion_from_euler([0.0, 0.0, 0.0], degrees=True)
        arm_controller.get_logger().info(f"Orientation goal: {orientation_goal}")

        arm_controller.go_to_pose(position_goal, orientation_goal)

        # # Run COM parameters estimation
        # arm_controller.run_com_parameters_estimation()

        # # Torques calibration
        # arm_controller.zero_torques()

        # # Stop the arm
        # response = arm_controller.stop_arm()

        rclpy.spin(arm_controller)

    except KeyboardInterrupt:
        arm_controller.get_logger().info("Keyboard interrupt")
        arm_controller.cancel_actions()

    finally:
        arm_controller.get_logger().info("Shutting down...")
        arm_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
