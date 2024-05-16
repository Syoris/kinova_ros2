#ifndef KINOVA_DRIVER_KINOVA_CARTESIAN_POSE_ACTION_H
#define KINOVA_DRIVER_KINOVA_CARTESIAN_POSE_ACTION_H

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

#include "kinova_driver/kinova_comm2.h"

#include "kinova_msgs/action/arm_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace kinova
{

class KinovaCartesianPoseActionServer
{
 public:
    using CartPoseAction = kinova_msgs::action::ArmPose;
    using GoalHandleCartPose = rclcpp_action::ServerGoalHandle<CartPoseAction>;

    // explicit KinovaCartesianPoseActionServer(rclcpp::Node::SharedPtr node, std::shared_ptr<KinovaComm2> kinova_comm2);
    explicit KinovaCartesianPoseActionServer(rclcpp::Node::SharedPtr node);

    ~KinovaCartesianPoseActionServer();

 private:
    // Action callbacks and attributes
    rclcpp_action::Server<CartPoseAction>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CartPoseAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCartPose> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleCartPose> goal_handle);

    void execute(const std::shared_ptr<GoalHandleCartPose> goal_handle);

    // Attributes
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<kinova::KinovaComm2> comm_;

    rclcpp::Time last_nonstall_time_;
    KinovaAngles last_nonstall_angles_;

    // Parameters
    std::string link_base_frame_;
    double rate_hz_;
    float position_tolerance_;
    float angle_tolerance_;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_CARTESIAN_POSE_ACTION_H