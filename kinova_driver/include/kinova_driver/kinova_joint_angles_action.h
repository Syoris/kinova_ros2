#ifndef KINOVA_DRIVER_KINOVA_JOINT_ANGLES_ACTION_H
#define KINOVA_DRIVER_KINOVA_JOINT_ANGLES_ACTION_H

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

#include "kinova_driver/kinova_comm2.h"

#include "kinova_msgs/action/arm_joint_angles.hpp"


namespace kinova
{

class KinovaAnglesActionServer
{
 public:
   using JointAnglesAction = kinova_msgs::action::ArmJointAngles;
   using GoalHandleJointAngles = rclcpp_action::ServerGoalHandle<JointAnglesAction>;

   explicit KinovaAnglesActionServer(rclcpp::Node::SharedPtr node, std::shared_ptr<KinovaComm2> kinova_comm2);
  //  explicit KinovaAnglesActionServer(rclcpp::Node::SharedPtr node);

   ~KinovaAnglesActionServer();

 private:
    // Action callbacks and attributes
    rclcpp_action::Server<JointAnglesAction>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointAnglesAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleJointAngles> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleJointAngles> goal_handle);

    void execute(const std::shared_ptr<GoalHandleJointAngles> goal_handle);

    // Attributes
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<kinova::KinovaComm2> comm_;

    // Parameters
    double rate_hz_;
    float tolerance_;
    double jointSpeedLimitJoints123;
    double jointSpeedLimitJoints456;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_JOINT_ANGLES_ACTION_H