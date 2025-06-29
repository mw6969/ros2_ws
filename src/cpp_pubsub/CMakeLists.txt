cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(talker src/publisher_lambda_function.cpp)
target_link_libraries(talker PUBLIC rclcpp::rclcpp ${tutorial_interfaces_TARGETS})

add_executable(listener src/subscriber_lambda_function.cpp)
target_link_libraries(listener PUBLIC rclcpp::rclcpp ${tutorial_interfaces_TARGETS})

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# Test support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosidl_default_runtime REQUIRED)

  ament_add_gtest(test_pubsub test/test_pubsub.cpp)
  if(TARGET test_pubsub)
    target_link_libraries(test_pubsub
      rclcpp::rclcpp
    )
    ament_target_dependencies(test_pubsub
      tutorial_interfaces
    )
  endif()
endif()

ament_package()