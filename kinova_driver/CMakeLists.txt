cmake_minimum_required(VERSION 3.8)
project(kinova_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


##### BUILD #####
# # Includes
# include_directories(
#   include
#   ${rclcpp_INCLUDE_DIRS}
#   # ${geometry_msgs_INCLUDE_DIRS}
#   # ${kinova_msgs_INCLUDE_DIRS}
#   # ${sensor_msgs_INCLUDE_DIRS}
#   # ${std_msgs_INCLUDE_DIRS}
#   # ${tf2_INCLUDE_DIRS}
#   # ${angles_INCLUDE_DIRS}
#   # ${interactive_markers_INCLUDE_DIRS}
#   # ${visualization_msgs_INCLUDE_DIRS}
# )

# kinova_driver_lib: Library with api, comm, ...
add_library(kinova_driver_lib
  src/kinova_api.cpp
  # src/kinova_arm.cpp
  src/kinova_comm2.cpp
  # src/kinova_tool_pose_action.cpp
  # src/kinova_joint_angles_action.cpp
  # src/kinova_fingers_action.cpp
  # src/kinova_ros_types.cpp
  # src/kinova_joint_trajectory_controller.cpp
  # include/kinova_driver/kinova_joint_trajectory_controller.h
  # include/kinova_driver/kinova_joint_angles_action.h
  # include/kinova_driver/kinova_api.h
  # include/kinova_driver/kinova_arm.h
  # include/kinova_driver/kinova_comm2.h
  # include/kinova_driver/kinova_fingers_action.h
  # include/kinova_driver/kinova_tool_pose_action.h
  # include/kinova_driver/kinova_ros_types.h
  # include/kinova/Kinova.API.USBCommLayerUbuntu.h
  # include/kinova/Kinova.API.USBCommandLayerUbuntu.h
  # include/kinova/KinovaTypes.h
)
ament_target_dependencies(kinova_driver_lib
  rclcpp
  # Add other dependencies if needed
)
target_include_directories(kinova_driver_lib PUBLIC
  include
)

# kinova_arm_driver
add_executable(kinova_arm_driver2
  src/nodes/kinova_arm_driver2.cpp
)
target_include_directories(kinova_arm_driver2 PUBLIC
  include)
ament_target_dependencies(kinova_arm_driver2
  # geometry_msgs
  # kinova_msgs
  rclcpp
  # sensor_msgs
  # std_msgs
  # tf2
  # interactive_markers
  # visualization_msgs
)
target_link_libraries(kinova_arm_driver2 kinova_driver_lib)


##### INSTALL #####
# Copy over Kinova .so binary blobs to /usr/lib
install(FILES
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommandLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommandLayerUbuntu.so
  DESTINATION lib
)


# Install executables and libraries
install(TARGETS 
  kinova_arm_driver2
  # kinova_tf_updater
  # kinova_interactive_control
  # kinova_driver
  kinova_driver_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

##### TESTING #####
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

##### AMENT PACKAGE #####
ament_export_dependencies(
  # geometry_msgs 
  # kinova_msgs
  rclcpp
  # rospy
  # sensor_msgs
  # std_msgs
  # tf2
)

# Declare public interface - What other pkg can use from this pkg
ament_export_include_directories(include)
# ament_export_libraries(kinova_driver)

ament_package()
