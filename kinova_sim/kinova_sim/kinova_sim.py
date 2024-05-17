from kinova_sim.KinovaSim import KinovaSim
import rclpy


def main():
    # Create a ROS node
    rclpy.init()
    kinova_sim = KinovaSim()

    # Spin the node
    rclpy.spin(kinova_sim)

    # Clean up
    kinova_sim.get_logger().info("Done, destroying node")
    kinova_sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
