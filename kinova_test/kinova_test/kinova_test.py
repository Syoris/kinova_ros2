import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionClient

import time

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
from kinova_msgs.action import ArmJointAngles


class KinovaTest(Node):
    def __init__(self):
        super().__init__("kinova_test")

        self._is_homed = False

        # self._init_clients()
        self._init_publishers()
        self._init_action_clients()

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

    # ---------------- ACTION CLIENTS ----------------
    # MARK: ACTION CLIENTS
    def _init_action_clients(self):
        self.get_logger().info("Creating action clients...")

        self._go_to_angles_client = ActionClient(
            self, ArmJointAngles, "/arm/go_to_angles"
        )

    def go_to_angles(self, angles):
        """Action

        Args:
            angles (list): Target angle of each joint
        """
        goal_msg = ArmJointAngles.Goal()
        goal_msg.angles.joint1 = angles[0]
        goal_msg.angles.joint2 = angles[1]
        goal_msg.angles.joint3 = angles[2]
        goal_msg.angles.joint4 = angles[3]
        goal_msg.angles.joint5 = angles[4]
        goal_msg.angles.joint6 = angles[5]
        goal_msg.angles.joint7 = angles[6]

        self._go_to_angles_client.wait_for_server()

        # self._go_to_angles_client.send_goal(goal_msg)

        # self._action_client.send_goal_async(goal_msg)

        self.get_logger().info("Sending goal...")
        self._go_to_angles_send_future = self._go_to_angles_client.send_goal_async(
            goal_msg, feedback_callback=self.go_to_angles_feedback_callback
        )
        self._go_to_angles_send_future.add_done_callback(
            self.go_to_angles_response_callback
        )

    def go_to_angles_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self._go_to_angles_goal_handle = goal_handle

        self.get_logger().info("Goal accepted :)")

        self._go_to_angles_goal_future = goal_handle.get_result_async()
        self._go_to_angles_goal_future.add_done_callback(
            self.go_to_angles_result_callback
        )

        # Start a 2 second timer
        self._cancel_timer = self.create_timer(2.0, self.cancel_timer_callback)

    def go_to_angles_result_callback(self, future: rclpy.task.Future):
        self.get_logger().info("Go to angles result callback")
        result = future.result()

        status = result.status
        self.get_logger().info(f"Status: {status}")

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Result: {result.result.angles}")

        else:
            self.get_logger().info(f"Goal failed with status: {status}")

    def go_to_angles_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.angles
        feedback_str = (
            f"Feedback:"
            f"\n\tJoint 1: {feedback.joint1}"
            f"\n\tJoint 2: {feedback.joint2}"
            f"\n\tJoint 3: {feedback.joint3}"
            f"\n\tJoint 4: {feedback.joint4}"
            f"\n\tJoint 5: {feedback.joint5}"
            f"\n\tJoint 6: {feedback.joint6}"
            f"\n\tJoint 7: {feedback.joint7}"
        )

        self.get_logger().info(f"{feedback_str}")

    def go_to_angles_cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
            # cancel_state = cancel_response.result.angles
            # _str = (
            #     f"Cancel state:"
            #     f"\n\tJoint 1: {cancel_state.joint1}"
            #     f"\n\tJoint 2: {cancel_state.joint2}"
            #     f"\n\tJoint 3: {cancel_state.joint3}"
            #     f"\n\tJoint 4: {cancel_state.joint4}"
            #     f"\n\tJoint 5: {cancel_state.joint5}"
            #     f"\n\tJoint 6: {cancel_state.joint6}"
            #     f"\n\tJoint 7: {cancel_state.joint7}"
            # )
            self.get_logger().info(f"{future}")

        else:
            self.get_logger().info("Goal failed to cancel")

    def cancel_timer_callback(self):
        self.get_logger().info("Canceling goal")
        # Cancel the goal
        future = self._go_to_angles_goal_handle.cancel_goal_async()
        future.add_done_callback(self.go_to_angles_cancel_done)

        # Cancel the timer
        self._cancel_timer.cancel()


def main():
    rclpy.init()

    kinova_test = KinovaTest()

    # # Start the arm
    # kinova_test.start_arm()

    # # Stop the arm
    # response = kinova_test.stop_arm()

    # # Home the arm
    # kinova_test.home_arm()
    # time.sleep(1)

    # # Pusblish joint vels
    # kinova_test.publish_joint_vels([])

    # Go to angles
    angles = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.2]
    kinova_test.go_to_angles(angles)

    rclpy.spin(kinova_test)

    kinova_test.get_logger().info("Done, destroying node")
    kinova_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
