cmake_minimum_required(VERSION 3.8)
project(kinova_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(kinova_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(sensor_msgs REQUIRED)


##### BUILD #####
# MARK: BUILD
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
  src/kinova_arm2.cpp
  src/kinova_comm2.cpp
  src/kinova_ros2_types.cpp
  src/kinova_joint_angles_action.cpp
  src/kinova_cartesian_pose_action.cpp
  src/kinova_arm_kinematics.cpp
)
ament_target_dependencies(kinova_driver_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  kinova_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  angles
)
target_include_directories(kinova_driver_lib PUBLIC
  include
)

# kinova_arm_driver
add_executable(kinova_arm_node
  src/nodes/kinova_arm_node.cpp
)
target_include_directories(kinova_arm_node PUBLIC
  include)
ament_target_dependencies(kinova_arm_node
  # geometry_msgs
  # kinova_msgs
  rclcpp
  tf2_geometry_msgs
  # sensor_msgs
  # std_msgs
  # tf2
  # interactive_markers
  # visualization_msgs
)
target_link_libraries(kinova_arm_node kinova_driver_lib)

# tf_updater
add_executable(kinova_tf_updater
  src/nodes/kinova_tf_updater.cpp
)
target_include_directories(kinova_tf_updater PUBLIC
  include)
ament_target_dependencies(kinova_tf_updater
  # geometry_msgs
  kinova_msgs
  rclcpp
  # sensor_msgs
  # std_msgs
  tf2
  # interactive_markers
  # visualization_msgs
)
target_link_libraries(kinova_tf_updater kinova_driver_lib)

##### INSTALL #####
# MARK: INSTALL
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
  kinova_arm_node
  kinova_tf_updater
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
  kinova_msgs
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
