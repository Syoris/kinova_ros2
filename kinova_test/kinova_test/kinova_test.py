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
import rclpy
from rclpy.node import Node

import time


class KinovaTest(Node):
    def __init__(self):
        super().__init__("kinova_test")

        self._is_homed = False

        self._init_clients()
        self._init_publishers()

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("KinovaTest node has been initialized.")

    # ---------------- CLIENTS ----------------
    # MARK: CLIENTS
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

    def publish_joint_vels(self, joint_vels):
        joint_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        msg = JointVelocity()
        msg.joint1 = joint_vels[0]
        msg.joint2 = joint_vels[1]
        msg.joint3 = joint_vels[2]
        msg.joint4 = joint_vels[3]
        msg.joint5 = joint_vels[4]
        msg.joint6 = joint_vels[5]
        msg.joint7 = joint_vels[6]

        self.joint_vels_pub_.publish(msg)

    def publish_pose_vels(self, pose_vels):
        pose_vels = [5.0, 0.0, 0.0, 0.0, 0.0, 5.0]

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

        self.get_logger().info("Timer callback")
        # self.publish_pose_vels([])
        self.publish_joint_vels([])


def main():
    rclpy.init()

    kinova_test = KinovaTest()

    # Start the arm
    kinova_test.start_arm()

    # # Stop the arm
    # response = kinova_test.stop_arm()

    # Home the arm
    kinova_test.home_arm()
    time.sleep(1)

    # Pusblish joint vels
    # kinova_test.publish_joint_vels([])

    rclpy.spin(kinova_test)

    kinova_test.get_logger().info("Done, destroying node")
    kinova_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
