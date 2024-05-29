cmake_minimum_required(VERSION 3.5)
project(jsn_sr04t_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(jsn_sr04t_node src/jsn_sr04t_node.cpp)
ament_target_dependencies(jsn_sr04t_node rclcpp rclcpp_components sensor_msgs)

install(TARGETS
  jsn_sr04t_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
