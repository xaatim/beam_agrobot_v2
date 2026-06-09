cmake_minimum_required(VERSION 3.8)
project(moveit_config)

find_package(ament_cmake REQUIRED)

# Install config and launch directories so ROS 2 can find them
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()