/*
 * kinova_arm_driver.h
 *
 *  Created on: Feb 26, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 *
 */

#ifndef KINOVA_DRIVER_KINOVA_ARM_H
#define KINOVA_DRIVER_KINOVA_ARM_H

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <kinova_msgs/srv/stop.hpp>
#include <kinova_msgs/srv/start.hpp>
#include <kinova_msgs/srv/home_arm.hpp>
#include <kinova_msgs/srv/set_force_control_params.hpp>
#include <kinova_msgs/srv/set_end_effector_offset.hpp>
#include <kinova_msgs/srv/set_null_space_mode_state.hpp>
#include <kinova_msgs/srv/set_torque_control_mode.hpp>
#include <kinova_msgs/srv/set_torque_control_parameters.hpp>
#include <kinova_msgs/srv/clear_trajectories.hpp>
#include <kinova_msgs/srv/add_pose_to_cartesian_trajectory.hpp>
#include <kinova_msgs/srv/zero_torques.hpp>
#include <kinova_msgs/srv/run_com_parameters_estimation.hpp>

#include <kinova_msgs/msg/joint_velocity.h>
#include <kinova_msgs/msg/pose_velocity.h>
#include <kinova_msgs/msg/pose_velocity_with_fingers.h>
#include <kinova_msgs/msg/pose_velocity_with_finger_velocity.h>
#include <kinova_msgs/msg/joint_torque.h>
#include <kinova_msgs/msg/finger_position.h>
#include <kinova_msgs/msg/joint_angles.h>
#include <kinova_msgs/msg/kinova_pose.h>
#include <kinova_msgs/msg/cartesian_force.h>

#include <time.h>
#include <math.h>
#include <vector>

#include "kinova/KinovaTypes.h"
#include "kinova_driver/kinova_comm2.h"
#include "kinova_driver/kinova_api.h"


namespace kinova
{

struct robot_info
{
    int id;
    std::string name;
    std::string type;
    std::string serial;
};

class KinovaArm2
{
 public:
    KinovaArm2(rclcpp::Node::SharedPtr node, std::shared_ptr<KinovaComm2> kinova_comm2, const std::string &kinova_robotType, const std::string &kinova_robotName);
    ~KinovaArm2();

    // //Subscriber callbacks --------------------------------------------------------
    // void jointVelocityCallback(const kinova_msgs::JointVelocityConstPtr& joint_vel);
    // void cartesianVelocityCallback(const kinova_msgs::PoseVelocityConstPtr& cartesian_vel);
    // void cartesianVelocityWithFingersCallback(const kinova_msgs::PoseVelocityWithFingersConstPtr& cartesian_vel_with_fingers);
    // void cartesianVelocityWithFingerVelocityCallback(const kinova_msgs::PoseVelocityWithFingerVelocityConstPtr& cartesian_vel_with_fingers);
    // void jointTorqueSubscriberCallback(const kinova_msgs::JointTorqueConstPtr& joint_torque);
    // void forceSubscriberCallback(const kinova_msgs::CartesianForceConstPtr& force);

    // Service callbacks -----------------------------------------------------------    
    void stopServiceCallback(const std::shared_ptr<kinova_msgs::srv::Stop::Request> request,  std::shared_ptr<kinova_msgs::srv::Stop::Response> res);

    void startServiceCallback(const std::shared_ptr<kinova_msgs::srv::Start::Request> request, std::shared_ptr<kinova_msgs::srv::Start::Response> res);
    
    void homeArmServiceCallback(const std::shared_ptr<kinova_msgs::srv::HomeArm::Request> request, 
    std::shared_ptr<kinova_msgs::srv::HomeArm::Response> res);

    void activateNullSpaceModeCallback(const std::shared_ptr<kinova_msgs::srv::SetNullSpaceModeState::Request> request,
                                       std::shared_ptr<kinova_msgs::srv::SetNullSpaceModeState::Response> res);

    void addCartesianPoseToTrajectory(const std::shared_ptr<kinova_msgs::srv::AddPoseToCartesianTrajectory::Request> request, std::shared_ptr<kinova_msgs::srv::AddPoseToCartesianTrajectory::Response> res);

    void clearTrajectoriesServiceCallback(const std::shared_ptr<kinova_msgs::srv::ClearTrajectories::Request> request,
                                          std::shared_ptr<kinova_msgs::srv::ClearTrajectories::Response> res);

    void setEndEffectorOffsetCallback(const std::shared_ptr<kinova_msgs::srv::SetEndEffectorOffset::Request> request,
                                      std::shared_ptr<kinova_msgs::srv::SetEndEffectorOffset::Response> res);                            

    // //Torque control --------------------------------------------------------------
    // bool setForceControlParamsCallback(kinova_msgs::SetForceControlParams::Request &req,
    //                                    kinova_msgs::SetForceControlParams::Response &res);
    // bool startForceControlCallback(kinova_msgs::Start::Request &req,
    //                                kinova_msgs::Start::Response &res);
    // bool stopForceControlCallback(kinova_msgs::Stop::Request &req,
    //                               kinova_msgs::Stop::Response &res);

    // bool setTorqueControlModeService(kinova_msgs::SetTorqueControlMode::Request &req,
    //                                  kinova_msgs::SetTorqueControlMode::Response &res);
    // bool setTorqueControlParametersService(kinova_msgs::SetTorqueControlParameters::Request &req,
    //                                        kinova_msgs::SetTorqueControlParameters::Response &res);

    void setJointTorquesToZeroCallback(const std::shared_ptr<kinova_msgs::srv::ZeroTorques::Request> request,
                                      std::shared_ptr<kinova_msgs::srv::ZeroTorques::Response> res);      

    void runCOMParameterEstimationCallback(const std::shared_ptr<kinova_msgs::srv::RunCOMParametersEstimation::Request> request, std::shared_ptr<kinova_msgs::srv::RunCOMParametersEstimation::Response> res);   
                             

 private:
    // void positionTimer(const ros::TimerEvent&);
    // void cartesianVelocityTimer(const ros::TimerEvent&);
    // void jointVelocityTimer(const ros::TimerEvent&);
    void statusTimer();

    void publishJointAngles(void);
    void publishToolPosition(void);
    void publishToolWrench(void);
    void publishFingerPosition(void);

    // tf::TransformListener tf_listener_;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<KinovaComm2> kinova_comm_;

    // Publishers, subscribers, services
    // ros::Subscriber joint_velocity_subscriber_;
    // ros::Subscriber cartesian_velocity_subscriber_;
    // ros::Subscriber cartesian_velocity_with_fingers_subscriber_;
    // ros::Subscriber cartesian_velocity_with_finger_velocity_subscriber_;
    // ros::Subscriber joint_torque_subscriber_;
    // ros::Subscriber cartesian_force_subscriber_;

    rclcpp::Publisher<kinova_msgs::msg::JointAngles>::SharedPtr joint_angles_publisher_;
    rclcpp::Publisher<kinova_msgs::msg::JointAngles>::SharedPtr joint_torque_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_position_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tool_wrench_publisher_;
    rclcpp::Publisher<kinova_msgs::msg::FingerPosition>::SharedPtr finger_position_publisher_;

    rclcpp::Publisher<kinova_msgs::msg::JointAngles>::SharedPtr joint_command_publisher_;
    rclcpp::Publisher<kinova_msgs::msg::KinovaPose>::SharedPtr cartesian_command_publisher_;

    // Services
    rclcpp::Service<kinova_msgs::srv::Stop>::SharedPtr stop_service_;
    rclcpp::Service<kinova_msgs::srv::Start>::SharedPtr start_service_;
    rclcpp::Service<kinova_msgs::srv::HomeArm>::SharedPtr homing_service_;
    rclcpp::Service<kinova_msgs::srv::SetNullSpaceModeState>::SharedPtr start_null_space_service_;
    rclcpp::Service<kinova_msgs::srv::AddPoseToCartesianTrajectory>::SharedPtr add_trajectory_;
    rclcpp::Service<kinova_msgs::srv::ClearTrajectories>::SharedPtr clear_trajectories_;

    // ros::ServiceServer set_torque_control_mode_service_;
    // ros::ServiceServer set_torque_control_parameters_service_;
    rclcpp::Service<kinova_msgs::srv::ZeroTorques>::SharedPtr set_actuator_torques_to_zero_;
    // ros::ServiceServer set_force_control_params_service_;
    // ros::ServiceServer start_force_control_service_;
    // ros::ServiceServer stop_force_control_service_;
    rclcpp::Service<kinova_msgs::srv::RunCOMParametersEstimation>::SharedPtr run_COM_parameter_estimation_service_;

    // ros::ServiceServer set_end_effector_offset_service_;

    // Timers for control loops
    // ros::Timer status_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Parameters
    std::string kinova_robotType_;
    std::string kinova_robotName_;
    std::string tf_prefix_;

    char robot_category_;
    int robot_category_version_;
    char wrist_type_;
    int arm_joint_number_;
    char robot_mode_;
    int finger_number_;
    int joint_total_number_;
    ROBOT_TYPE robot_type_;

    double status_interval_seconds_;
    double finger_conv_ratio_;
    bool convert_joint_velocities_;

    // State tracking or utility members
    AngularInfo joint_velocities_;
    float l_joint_torque_[COMMAND_SIZE];
    float l_force_cmd_[COMMAND_SIZE];
    CartesianInfo cartesian_velocities_;

    std::vector< std::string > joint_names_;

    //multiple robots
    int active_robot_id_;
    std::vector<robot_info> robots_;

};


}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_ARM_H
