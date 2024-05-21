cmake_minimum_required(VERSION 3.8)
project(kinova_description)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

 install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
