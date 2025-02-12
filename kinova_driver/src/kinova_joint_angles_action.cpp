#include "kinova_driver/kinova_joint_angles_action.h"
// #include "kinova_driver/kinova_ros_types.h"

namespace kinova
{

KinovaAnglesActionServer::KinovaAnglesActionServer(rclcpp::Node::SharedPtr node, std::shared_ptr<KinovaComm2> kinova_comm2):node_(node), comm_(kinova_comm2){
// KinovaAnglesActionServer::KinovaAnglesActionServer(rclcpp::Node::SharedPtr node):node_(node){
    
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<JointAnglesAction>(
      this->node_,
      "go_to_angles",
      std::bind(&KinovaAnglesActionServer::handle_goal, this, _1, _2),
      std::bind(&KinovaAnglesActionServer::handle_cancel, this, _1),
      std::bind(&KinovaAnglesActionServer::handle_accepted, this, _1));

    rate_hz_ = 1;
    tolerance_ = 1.0;
    jointSpeedLimitJoints123 = 20;
    jointSpeedLimitJoints456 = 20;
}


KinovaAnglesActionServer::~KinovaAnglesActionServer()
{
}

rclcpp_action::GoalResponse KinovaAnglesActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointAnglesAction::Goal> goal){
    // Log goal request with target joint angles
    RCLCPP_INFO(node_->get_logger(), "Received goal request with target joint angles: "
        "\n\t-J1: %f"
        "\n\t-J2: %f"
        "\n\t-J3: %f"
        "\n\t-J4: %f"
        "\n\t-J5: %f"
        "\n\t-J6: %f"
        "\n\t-J7: %f", goal->angles.joint1, goal->angles.joint2, goal->angles.joint3, 
        goal->angles.joint4, goal->angles.joint5, goal->angles.joint6, goal->angles.joint7);
        
    (void)uuid; // unused

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}


rclcpp_action::CancelResponse KinovaAnglesActionServer::handle_cancel(const std::shared_ptr<GoalHandleJointAngles> goal_handle){
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");

    (void)goal_handle; // unused

    return rclcpp_action::CancelResponse::ACCEPT;
}


void KinovaAnglesActionServer::handle_accepted(const std::shared_ptr<GoalHandleJointAngles> goal_handle){
    using namespace std::placeholders;

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&KinovaAnglesActionServer::execute, this, _1), goal_handle}.detach();
}


void KinovaAnglesActionServer::execute(const std::shared_ptr<GoalHandleJointAngles> goal_handle){
    RCLCPP_INFO(node_->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<JointAnglesAction::Feedback>();
    auto & curr_angles_msg = feedback->angles;
    
    auto result = std::make_shared<JointAnglesAction::Result>();

    KinovaAngles current_joint_angles;
    KinovaAngles target(goal->angles);

    RCLCPP_INFO(node_->get_logger(), "Target joint angles: %f, %f, %f, %f, %f, %f, %f", target.Actuator1, target.Actuator2, target.Actuator3, target.Actuator4, target.Actuator5, target.Actuator6, target.Actuator7);

    comm_->setJointAngles(target, jointSpeedLimitJoints123, jointSpeedLimitJoints456);

    try{
        while (rclcpp::ok())
        {
            comm_->getJointAngles(current_joint_angles);
            curr_angles_msg = current_joint_angles.constructAnglesMsg();

            
            // Check if the action has succeeeded
            if (target.isCloseToOther(current_joint_angles, tolerance_))
            {
                result->angles = curr_angles_msg;
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
                return;
            }

            // Check if there is a cancel request
            else if (goal_handle->is_canceling()) {
                result->angles = curr_angles_msg;
                
                goal_handle->canceled(result);
                
                comm_->eraseAllTrajectories();
                comm_->stopAPI();

                RCLCPP_INFO(node_->get_logger(), "Goal canceled");
                return;
            }
            
            // Arm stopped
            else if (comm_->isStopped())
            {
                RCLCPP_INFO(node_->get_logger(), "Could not complete action because the arm is 'stopped'.");
                result->angles = curr_angles_msg;

                goal_handle->abort(result);
                RCLCPP_INFO(node_->get_logger(), "Goal aborted");

                return;
            }

            // Feedback
            else{
                // Update sequence
                // curr_angles_msg = current_joint_angles.constructAnglesMsg();

                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(node_->get_logger(), "Publish feedback");
            }

            // Abort example
            // {
            //     result->angles = curr_angles_msg;
            //     goal_handle->abort(result);
            //     RCLCPP_INFO(node_->get_logger(), "Goal aborted");
            //     return;
            // }

            loop_rate.sleep();
        }
    }
    catch(const std::exception& e)
    {
        comm_->eraseAllTrajectories();

        result->angles = curr_angles_msg;
        RCLCPP_ERROR(node_->get_logger(), "Error executing goal: %s", e.what());
        RCLCPP_ERROR(node_->get_logger(), "Aborting goal");
        goal_handle->abort(result);
    }

}

} // namespace kinova
