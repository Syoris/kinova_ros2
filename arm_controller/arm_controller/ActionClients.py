from abc import ABC
import rclpy
from rclpy.action import ActionClient

# from rclpy.rate import Rate
from rclpy.duration import Duration  # Add this at the top of your file

from action_msgs.msg import GoalStatus

from threading import Event


class MyActionClient(ABC):
    def __init__(self, node, action_type, action_name: str, wait_for_result=True):
        self._node = node
        self._action_name = action_name
        self._action_type = action_type
        self._wait_for_result = wait_for_result

        self._goal_handle = None
        self._send_goal_future = None
        self._result_future = None

        self.action_done_event = Event()

        self._client = ActionClient(
            self._node, action_type=self._action_type, action_name=self._action_name
        )

    def send_goal(self, goal_msg):
        """Send goal to action server

        Args:
            goal_msg (_type_): Message matching the goal type of the action server
        """
        self._node.get_logger().info("Sending goal...")

        self.action_done_event.clear()

        self._client.wait_for_server()

        self._send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        self._send_goal_future.add_done_callback(self._goal_response_cb)

        # while rclpy.ok() and self._result_future is None:
        #     self._node.get_logger().info("Waiting...")
        #     rclpy.spin_once(self._node, timeout_sec=0.1)

        # self._node.get_logger().info("Waiting done!")

        # return self._result_future

    def cancel_goal(self):
        self._node.get_logger().info("Canceling goal")

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
            self._node.get_logger().info("Goal rejected :(")
            return

        self._node.get_logger().info("Goal accepted :)")

        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._get_result_cb)

    def _get_result_cb(self, future: rclpy.task.Future):
        self._node.get_logger().info("Result callback")
        result = future.result()

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info("GOAL SUCCESSFUL!")

        elif status == GoalStatus.STATUS_ABORTED:
            self._node.get_logger().info("GOAL ABORTED")

        elif status == GoalStatus.STATUS_CANCELED:
            self._node.get_logger().info("GOAL CANCELED")

        else:
            self._node.get_logger().info(f"Goal failed with status: {status}")

        self._node.get_logger().info(f"{self._process_result_msg(result.result)}")

        # self._result_future = None

    def _feedback_cb(self, feedback_msg):
        feedback_str = f"{self._process_feedback_msg(feedback_msg.feedback)}"

        self._node.get_logger().info(f"{feedback_str}")

    def _cancel_done_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self._node.get_logger().info("Goal successfully canceled")

        else:
            self._node.get_logger().info("Goal failed to cancel")

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
    def __init__(self, node, action_type, action_name: str):
        super().__init__(node, action_type, action_name)

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


class GoToPoseClient(MyActionClient):
    def __init__(self, node, action_type, action_name: str):
        super().__init__(node, action_type, action_name)

    def _process_result_msg(self, result_msg):
        result_str = (
            f"[Result] Current pose:"
            f"\nPosition"
            f"\n\t-x: {result_msg.pose.pose.position.x}"
            f"\n\t-y: {result_msg.pose.pose.position.y}"
            f"\n\t-z: {result_msg.pose.pose.position.z}"
            f"\nOrientation"
            f"\n\t-x: {result_msg.pose.pose.orientation.x}"
            f"\n\t-y: {result_msg.pose.pose.orientation.y}"
            f"\n\t-z: {result_msg.pose.pose.orientation.z}"
            f"\n\t-w: {result_msg.pose.pose.orientation.w}"
        )

        return result_str

    def _process_feedback_msg(self, feedback_msg):
        feedback_str = (
            f"[Feedback] Current pose:"
            f"\nPosition"
            f"\n\t-x: {feedback_msg.pose.pose.position.x}"
            f"\n\t-y: {feedback_msg.pose.pose.position.y}"
            f"\n\t-z: {feedback_msg.pose.pose.position.z}"
            f"\nOrientation"
            f"\n\t-x: {feedback_msg.pose.pose.orientation.x}"
            f"\n\t-y: {feedback_msg.pose.pose.orientation.y}"
            f"\n\t-z: {feedback_msg.pose.pose.orientation.z}"
            f"\n\t-w: {feedback_msg.pose.pose.orientation.w}"
        )

        return feedback_str
