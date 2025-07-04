cmake_minimum_required(VERSION 3.8)
project(bag_recorder_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
target_link_libraries(simple_bag_recorder rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp ${std_msgs_TARGETS})

add_executable(data_generator_node src/data_generator_node.cpp)
target_link_libraries(data_generator_node PUBLIC rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp ${example_interfaces_TARGETS})

add_executable(data_generator_executable src/data_generator_executable.cpp)
target_link_libraries(data_generator_executable PUBLIC rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp ${example_interfaces_TARGETS})

install(TARGETS
  data_generator_executable
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  data_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
