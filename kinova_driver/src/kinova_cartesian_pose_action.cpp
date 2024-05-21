#include "kinova_driver/kinova_cartesian_pose_action.h"
// #include "kinova_driver/kinova_ros_types.h"

namespace kinova
{

KinovaCartesianPoseActionServer::KinovaCartesianPoseActionServer(rclcpp::Node::SharedPtr node, std::shared_ptr<KinovaComm2> kinova_comm2):node_(node), comm_(kinova_comm2){
// KinovaCartesianPoseActionServer::KinovaCartesianPoseActionServer(rclcpp::Node::SharedPtr node):node_(node){
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<CartPoseAction>(
      this->node_,
      "go_to_pose",
      std::bind(&KinovaCartesianPoseActionServer::handle_goal, this, _1, _2),
      std::bind(&KinovaCartesianPoseActionServer::handle_cancel, this, _1),
      std::bind(&KinovaCartesianPoseActionServer::handle_accepted, this, _1));

    link_base_frame_ = "arm_base_link";
    rate_hz_ = 1;
    angle_tolerance_ = 2.0*M_PI/180;
    position_tolerance_ = 0.01;
}


KinovaCartesianPoseActionServer::~KinovaCartesianPoseActionServer()
{
}

rclcpp_action::GoalResponse KinovaCartesianPoseActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CartPoseAction::Goal> goal){
    geometry_msgs::msg::Pose goal_pose = goal->pose.pose;

    // Log goal request with target joint angles
    RCLCPP_INFO(node_->get_logger(), "Received goal request with target pose: "
        "\n\t-x: %f"
        "\n\t-y: %f"
        "\n\t-z: %f"
        "\n\t-r: %f"
        "\n\t-p: %f"
        "\n\t-y: %f", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z);
        
    (void)uuid; // unused

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}


rclcpp_action::CancelResponse KinovaCartesianPoseActionServer::handle_cancel(const std::shared_ptr<GoalHandleCartPose> goal_handle){
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");

    (void)goal_handle; // unused

    return rclcpp_action::CancelResponse::ACCEPT;
}


void KinovaCartesianPoseActionServer::handle_accepted(const std::shared_ptr<GoalHandleCartPose> goal_handle){
    using namespace std::placeholders;

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&KinovaCartesianPoseActionServer::execute, this, _1), goal_handle}.detach();
}


void KinovaCartesianPoseActionServer::execute(const std::shared_ptr<GoalHandleCartPose> goal_handle){
    RCLCPP_INFO(node_->get_logger(), "Executing goal");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CartPoseAction::Feedback>();
    auto result = std::make_shared<CartPoseAction::Result>();

    feedback->pose.header.frame_id = goal->pose.header.frame_id;
    result->pose.header.frame_id = goal->pose.header.frame_id;

    // rclcpp::Time current_time = node_->now();

    KinovaPose current_pose;
    geometry_msgs::msg::PoseStamped local_pose;
    local_pose.header.frame_id = link_base_frame_;
    rclcpp::Rate loop_rate(rate_hz_);


    // Execute the action
    try{
        // Process goal frame
        // listener.transformPose(local_pose.header.frame_id, goal->pose, local_pose); // TODO: Transform TF
        // comm_->getCartesianPosition(current_pose);
        local_pose.pose = goal->pose.pose;

        KinovaPose target(local_pose.pose);
        RCLCPP_INFO(node_->get_logger(), "Target pose (arm frame): "
            "\n\t-x: %f"
            "\n\t-y: %f"
            "\n\t-z: %f"
            "\n\t-r: %f"
            "\n\t-p: %f"
            "\n\t-y: %f", target.X, target.Y, target.Z, target.ThetaX, target.ThetaY, target.ThetaZ);


        while (rclcpp::ok())
        {
            // comm_->setCartesianPosition(target);

            // rclcpp::spin_some(node_);

            // Get current pose
            // comm_->getCartesianPosition(current_pose);
            // current_time = node_->now();

            local_pose.pose = current_pose.constructPoseMsg();

            // TODO: Transform TF
            // listener.transformPose(feedback.pose.header.frame_id, local_pose, feedback.pose);
            feedback->pose = local_pose;



            // Check conditions
            if (target.isCloseToOther(current_pose, position_tolerance_, angle_tolerance_))
            {
                RCLCPP_INFO(node_->get_logger(), "Goal reached");
                result->pose = feedback->pose;
                goal_handle->succeed(result);
                return;
            }

            // Cancel request
            else if (goal_handle->is_canceling()) {
                result->pose = feedback->pose;
                
                goal_handle->canceled(result);
                RCLCPP_INFO(node_->get_logger(), "Goal canceled");
                return;
            }

            // Arm stopped
            else if (comm_->isStopped())
            {
                RCLCPP_INFO(node_->get_logger(), "Could not complete cartesian action because the arm is 'stopped'.");
                result->pose = feedback->pose;

                goal_handle->abort(result);
                RCLCPP_INFO(node_->get_logger(), "Goal aborted");

                return;
            }

            // Feedback
            else{
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(node_->get_logger(), "[Feedback] Target pose (arm frame): "
                    "\n\t-x: %f"
                    "\n\t-y: %f"
                    "\n\t-z: %f"
                    "\n\t-r: %f"
                    "\n\t-p: %f"
                    "\n\t-y: %f", current_pose.X, current_pose.Y, current_pose.Z, current_pose.ThetaX, current_pose.ThetaY, current_pose.ThetaZ);
            }

            loop_rate.sleep();
        }
        


    }
    catch(const std::exception& e)
    {
        result->pose = feedback->pose;
        RCLCPP_ERROR(node_->get_logger(), "Error executing goal: %s", e.what());
        RCLCPP_ERROR(node_->get_logger(), "Aborting goal");
        goal_handle->abort(result);
    }

}

} // namespace kinova
