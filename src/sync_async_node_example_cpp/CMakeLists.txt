cmake_minimum_required(VERSION 3.8)
project(sync_async_node_example_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(SyncAsyncWriter src/sync_async_writer.cpp)
target_link_libraries(SyncAsyncWriter PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(SyncAsyncReader src/sync_async_reader.cpp)
target_link_libraries(SyncAsyncReader PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(ping_service src/ping_service.cpp)
target_link_libraries(ping_service PUBLIC ${example_interfaces_TARGETS} rclcpp::rclcpp)

add_executable(ping_client src/ping_client.cpp)
target_link_libraries(ping_client PUBLIC ${example_interfaces_TARGETS} rclcpp::rclcpp)

install(TARGETS
  SyncAsyncWriter
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  SyncAsyncReader
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ping_service
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ping_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
