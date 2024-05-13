cmake_minimum_required(VERSION 3.8)
project(kinova_description)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)


install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes
   DESTINATION share/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.dae"
 )

ament_package()
