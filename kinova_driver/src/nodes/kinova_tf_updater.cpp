#include "rclcpp/rclcpp.hpp"

#include "kinova_msgs/msg/joint_angles.hpp"


using namespace std::chrono_literals;

namespace kinova
{

class KinovaTfUpdater : public rclcpp::Node
{
public:
    KinovaTfUpdater(): Node("kinova_tf_updater")
    {
        using std::placeholders::_1;
        joint_angles_subscriber_ = this->create_subscription<kinova_msgs::msg::JointAngles>(
            "/arm/out/joint_angles", 10, std::bind(&KinovaTfUpdater::joint_angles_msg_cb, this, _1));

        // Init angles
        current_angles_.joint1 = 0;
        current_angles_.joint2 = 0;
        current_angles_.joint3 = 0;
        current_angles_.joint4 = 0;
        current_angles_.joint5 = 0;
        current_angles_.joint6 = 0;
        current_angles_.joint7 = 0;

        tf_update_timer_ = this->create_wall_timer(10ms, std::bind(&KinovaTfUpdater::tfUpdateHandler, this));

        RCLCPP_INFO(this->get_logger(), "kinova_tf_updater ready");

    }

private:
    void joint_angles_msg_cb(const kinova_msgs::msg::JointAngles & joint_angles){  
        // RCLCPP_INFO(this->get_logger(), "Received joint angles message");
        
        current_angles_.joint1 = joint_angles.joint1;
        current_angles_.joint2 = joint_angles.joint2;
        current_angles_.joint3 = joint_angles.joint3;
        current_angles_.joint4 = joint_angles.joint4;
        current_angles_.joint5 = joint_angles.joint5;
        current_angles_.joint6 = joint_angles.joint6;
        current_angles_.joint7 = joint_angles.joint7;

    }

    void calculatePostion(void){
        // // Update the forward Kinematics
        // float Q[7] = {kinematics_.degToRad(current_angles_.joint1),
        //             kinematics_.degToRad(current_angles_.joint2),
        //             kinematics_.degToRad(current_angles_.joint3),
        //             kinematics_.degToRad(current_angles_.joint4),
        //             kinematics_.degToRad(current_angles_.joint5),
        //             kinematics_.degToRad(current_angles_.joint6),
        //             kinematics_.degToRad(current_angles_.joint7)};

        // kinematics_.updateForward(Q);

    }

    void tfUpdateHandler(){
        this->calculatePostion();  // Update TF Tree
    }

    // params    
    // std::shared_ptr<kinova::KinovaForwardKin> forward_kin_;
    kinova_msgs::msg::JointAngles current_angles_;
    rclcpp::Subscription<kinova_msgs::msg::JointAngles>::SharedPtr joint_angles_subscriber_;
    rclcpp::TimerBase::SharedPtr tf_update_timer_;
};

} // namespace kinova

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto driver_node = std::make_shared<kinova::KinovaTfUpdater>();
//   driver_node->init_driver();
  
  rclcpp::spin(driver_node);
  rclcpp::shutdown();
  return 0;
}