import rclpy
from arm_controller.arm_controller import ArmController
from rclpy.executors import MultiThreadedExecutor

CONFIRM_ACTIONS = False


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    try:
        arm_controller = ArmController()
        arm_controller.get_logger().info("------ Starting demo ------")

        # # Start the arm
        # arm_controller.start_arm()

        ans = ""

        # # Home the arm
        # if CONFIRM_ACTIONS:
        #     ans = input("\nHoming the arm (n to skip)")
        # if ans.lower() != "n":
        #     arm_controller.home_arm()

        # Joint angles
        if CONFIRM_ACTIONS:
            ans = input("\nTesting joint angles (n to skip)")
        if ans.lower() != "n":
            angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            arm_controller.get_logger().info(f"Going to: {angles}")
            arm_controller.go_to_angles(angles)
            rclpy.spin_once(arm_controller)

        # # Cartesian position
        # if CONFIRM_ACTIONS:
        #     ans = input("\nGoing to cartesian position (n to skip)")
        # if ans.lower() != "n":
        #     position = [0.0, 0.0, 0.0]
        #     arm_controller.go_to_cartesian_position(position)

        # Joint velocity control
        if CONFIRM_ACTIONS:
            ans = input("\nJoint velocity control (n to skip)")
        if ans.lower() != "n":
            joint_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            arm_controller.get_logger().info(f"Setting joint vels to: {joint_vels}")
            arm_controller.publish_joint_vels(joint_vels)

        # Cartesian velocity control

        # # Pusblish joint vels
        # arm_controller.publish_joint_vels([])

        # Stop the arm
        arm_controller.stop_arm()

    except KeyboardInterrupt:
        arm_controller.get_logger().info("Demo interrupted")

    rclpy.spin_once(arm_controller, executor)

    arm_controller.get_logger().info("------ Demo completed ------")
    arm_controller.destroy_node()

    rclpy.shutdown()
