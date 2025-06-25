cmake_minimum_required(VERSION 3.8)
project(cpp_parameters)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(minimal_param_node src/cpp_parameters_node.cpp)
target_link_libraries(minimal_param_node rclcpp::rclcpp)

install(TARGETS
    minimal_param_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
