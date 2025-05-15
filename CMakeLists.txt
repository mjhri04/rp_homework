cmake_minimum_required(VERSION 3.8)
project(rp_homework)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Serviceinterface.srv"
  "action/Actioninterface.action"
  DEPENDENCIES std_msgs action_msgs
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_executable(node1 src/node1.cpp)
ament_target_dependencies(node1
  rclcpp
  std_msgs
  rclcpp_action
)
add_dependencies(node1 ${PROJECT_NAME})
target_link_libraries(node1 ${PROJECT_NAME}__rosidl_typesupport_cpp)

add_executable(node2 src/node2.cpp)
ament_target_dependencies(node2
  rclcpp
  std_msgs
  rclcpp_action
)
add_dependencies(node2 ${PROJECT_NAME})
target_link_libraries(node2 ${PROJECT_NAME}__rosidl_typesupport_cpp)

install(TARGETS
  node1
  node2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
