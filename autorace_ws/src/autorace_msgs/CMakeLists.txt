cmake_minimum_required(VERSION 3.8)
project(autorace_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/LaneCmd.msg"
  "msg/LaneState.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/TrafficLight.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
