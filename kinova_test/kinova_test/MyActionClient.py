from abc import ABC
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from kinova_msgs.action import ArmJointAngles


class MyActionClient(ABC):
    def __init__(self, node, action_name: str, action_type):
        self._node = node
        self._action_name = action_name

        self._client = ActionClient(self, action_type, action_name)

    def send_goal(self, goal_msg):
        """Send goal to action server

        Args:
            goal_msg (_type_): Message matching the goal type of the action server
        """
        self.node_.get_logger().info("Sending goal...")

        self._send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def cancel_goal(self):
        self.node_.get_logger().info("Canceling goal")

        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_done_cb)

    def _goal_response_cb(self, future):
        """Response of the action server to the goal sent

        Args:
            future (Future): Future object containing the goal handle
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node_.get_logger().info("Goal rejected :(")
            return

        self.node_.get_logger().info("Goal accepted :)")

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_cb)

    def _get_result_cb(self, future: rclpy.task.Future):
        self.node_.get_logger().info("Result callback")
        result = future.result()

        status = result.status
        self.get_logger().info(f"Status: {status}")

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("GOAL SUCCESSFUL!")

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("GOAL ABORTED")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("GOAL CANCELED")

        else:
            self.get_logger().info(f"Goal failed with status: {status}")

        self.node_.get_logger().info(
            f"Result msg: {self._process_result_msg(result.result)}"
        )

    def _feedback_cb(self, feedback_msg):
        feedback_str = f"Feedback: {self._process_feedback_msg(feedback_msg.feedback)}"

        self.node_.get_logger().info(f"{feedback_str}")

    def _cancel_done_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node_.get_logger().info("Goal successfully canceled")

        else:
            self.node_.get_logger().info("Goal failed to cancel")

    def _process_result_msg(self, result_msg) -> str:
        """Process the result message. Implement in child class.

        Args:
            result_msg (_type_): Message containing the result of the action server

        Returns:
            str: String representation of the result
        """

        return result_msg

    def _process_feedback_msg(self, feedback_msg) -> str:
        """Process the feedback message. Implement in child class.

        Args:
            feedback_msg (_type_): Message containing the feedback of the action server

        Returns:
            str: String representation of the feedback
        """

        return feedback_msg


class GoToAnglesClient(MyActionClient):
    def __init__(self, node, action_name, action_type):
        super().__init__(node, action_name, action_type)

    def _process_result_msg(self, result_msg):
        result_str = (
            f"[Feedback] Current angles:"
            f"\n\tJoint 1: {result_msg.angles.joint1}"
            f"\n\tJoint 2: {result_msg.angles.joint2}"
            f"\n\tJoint 3: {result_msg.angles.joint3}"
            f"\n\tJoint 4: {result_msg.angles.joint4}"
            f"\n\tJoint 5: {result_msg.angles.joint5}"
            f"\n\tJoint 6: {result_msg.angles.joint6}"
            f"\n\tJoint 7: {result_msg.angles.joint7}"
        )

        return result_str

    def _process_feedback_msg(self, feedback_msg):
        feedback_str = (
            f"[Feedback] Current angles:"
            f"\n\tJoint 1: {feedback_msg.angles.joint1}"
            f"\n\tJoint 2: {feedback_msg.angles.joint2}"
            f"\n\tJoint 3: {feedback_msg.angles.joint3}"
            f"\n\tJoint 4: {feedback_msg.angles.joint4}"
            f"\n\tJoint 5: {feedback_msg.angles.joint5}"
            f"\n\tJoint 6: {feedback_msg.angles.joint6}"
            f"\n\tJoint 7: {feedback_msg.angles.joint7}"
        )

        return feedback_str
