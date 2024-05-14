import sys

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


class KinovaTest(Node):
    def __init__(self):
        super().__init__("kinova_test")

        self._init_clients()

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
        self.future = self.start_cli.call_async(self.start_cli_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def stop_arm(self):
        self.get_logger().info("Stopping the arm...")
        self.future = self.stop_cli.call_async(self.stop_cli_req)
        rclpy.spin_until_future_complete(self, self.future)

        res = self.future.result()
        self.get_logger().info(f"Result: {res.stop_result}")

        return self.future.result()

    def home_arm(self):
        self.get_logger().info("Homing the arm...")
        try:
            self.future = self.home_arm_cli.call_async(self.home_arm_cli_req)
            rclpy.spin_until_future_complete(self, self.future)
            res = self.future.result()
            self.get_logger().info(f"Result: {res.homearm_result}")
            return res

        except KeyboardInterrupt:
            self.future.cancel()
            self.get_logger().info("Service call cancelled due to keyboard interrupt.")

            self.stop_arm()


def main():
    rclpy.init()

    kinova_test = KinovaTest()

    # # Start the arm
    # response = kinova_test.start_arm()
    # kinova_test.get_logger().info(f"Result: {response.start_result}")

    # # Stop the arm
    # response = kinova_test.stop_arm()

    # Home the arm
    kinova_test.home_arm()
    kinova_test.get_logger().info("Arm has been homed.")

    kinova_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
