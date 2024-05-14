cmake_minimum_required(VERSION 3.8)
project(kinova_msgs)

set(CMAKE_CXX_STANDARD 17)

# find_package(catkin REQUIRED COMPONENTS
#   message_generation
#   actionlib
#   actionlib_msgs
#   geometry_msgs
# )
find_package(ament_cmake REQUIRED)
# find_package(message_generation REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Num.msg"
#   "msg/Sphere.msg"
#   "srv/AddThreeInts.srv"
#   DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
# )

set(msg_files
  "msg/FingerPosition.msg"
  "msg/JointAngles.msg"
  "msg/JointVelocity.msg"
  "msg/JointTorque.msg"
  "msg/KinovaPose.msg"
  "msg/PoseVelocity.msg"
  "msg/PoseVelocityWithFingers.msg"
  "msg/PoseVelocityWithFingerVelocity.msg"
  "msg/CartesianForce.msg"
)

set(srv_files
  "srv/Start.srv"
  "srv/Stop.srv"
  "srv/HomeArm.srv"
  "srv/SetForceControlParams.srv"
  "srv/SetEndEffectorOffset.srv"
  "srv/SetNullSpaceModeState.srv"
  "srv/SetTorqueControlMode.srv"
  "srv/SetTorqueControlParameters.srv"
  "srv/ClearTrajectories.srv"
  "srv/ZeroTorques.srv"
  "srv/AddPoseToCartesianTrajectory.srv"
  "srv/RunCOMParametersEstimation.srv"
)

set(action_files
  "action/ArmJointAngles.action"
  "action/ArmPose.action"
  "action/ArmKinovaPose.action"
  "action/SetFingersPosition.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  # ${action_files}
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

# add_message_files(
#   FILES
#   FingerPosition.msg
#   JointAngles.msg
#   JointVelocity.msg
#   JointTorque.msg
#   KinovaPose.msg
#   PoseVelocity.msg
#   PoseVelocityWithFingers.msg
#   PoseVelocityWithFingerVelocity.msg
#   CartesianForce.msg
# )


# add_service_files(
#   FILES
#   Start.srv
#   Stop.srv
#   HomeArm.srv
#   SetForceControlParams.srv
#   SetEndEffectorOffset.srv
#   SetNullSpaceModeState.srv
#   SetTorqueControlMode.srv
#   SetTorqueControlParameters.srv
#   ClearTrajectories.srv
#   ZeroTorques.srv
#   AddPoseToCartesianTrajectory.srv
#   RunCOMParametersEstimation.srv
# )

# add_action_files(DIRECTORY action
#   FILES
#   ArmJointAngles.action
#   ArmPose.action
#   Arm_KinovaPose.action
#   SetFingersPosition.action
# )

# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs
#   geometry_msgs
# )

# catkin_package(
#   CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
# )

ament_package()

