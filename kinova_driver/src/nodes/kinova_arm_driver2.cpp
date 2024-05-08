//============================================================================
// Name        : kinova_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Kinova robotic manipulator arm
//============================================================================

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_comm2.h"
// #include "kinova_driver/kinova_arm.h"
// #include "kinova_driver/kinova_tool_pose_action.h"
// #include "kinova_driver/kinova_joint_angles_action.h"
// #include "kinova_driver/kinova_fingers_action.h"
// #include "kinova_driver/kinova_joint_trajectory_controller.h"


class KinovaArmDriver2 : public rclcpp::Node
{
public:
    KinovaArmDriver2(): Node("kinova_arm_driver2")
    {
        RCLCPP_INFO(this->get_logger(), "KinovaArmDriver2 constructor");

        // Parameters
        this->declare_parameter("kinova_robot_type", "TYPE");
        this->declare_parameter("kinova_robot_name", "ROBOT");
        this->declare_parameter("connection_type", "USB");
        this->declare_parameter("serial_number", "serial_number");
        
        kinova_robot_name_ = this->get_parameter("kinova_robot_name").as_string();
        kinova_robot_type_ = this->get_parameter("kinova_robot_type").as_string();

        RCLCPP_INFO(this->get_logger(), "kinova_robot_type is %s.", kinova_robot_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "kinova_robot_name is %s.", kinova_robot_name_.c_str());

        // try
        // {
        //     kinova::KinovaComm2 comm(&this, api_mutex, is_first_init, kinova_robot_type);
            // kinova::KinovaArm kinova_arm(comm, nh, kinova_robotType, kinova_robotName);
            // kinova::KinovaPoseActionServer pose_server(comm, nh, kinova_robotType, kinova_robotName);
            // kinova::KinovaAnglesActionServer angles_server(comm, nh);
            // kinova::KinovaFingersActionServer fingers_server(comm, nh);
            // kinova::JointTrajectoryController joint_trajectory_controller(comm, nh);
            // ros::spin();
        // }
        // catch(const std::exception& e)
        // {
        //     ROS_ERROR_STREAM(e.what());
        //     kinova::KinovaAPI api;
        //     boost::recursive_mutex::scoped_lock lock(api_mutex);
        //     api.closeAPI();
        //     ros::Duration(1.0).sleep();
        // }

        is_first_init_ = false;
    }

    void init_driver(){
        RCLCPP_DEBUG(this->get_logger(), "KinovaArmDriver2 init_driver");

        comm_ = std::make_unique<kinova::KinovaComm2>(this->shared_from_this(), api_mutex_, is_first_init_, kinova_robot_type_);
    }

private:
    boost::recursive_mutex api_mutex_;
    bool is_first_init_ = true;

    // params
    std::string kinova_robot_name_;
    std::string kinova_robot_type_;
    
    // Classes
    std::unique_ptr<kinova::KinovaComm2> comm_;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto driver_node = std::make_shared<KinovaArmDriver2>();
  driver_node->init_driver();
  
  rclcpp::spin(driver_node);
  rclcpp::shutdown();
  return 0;
}