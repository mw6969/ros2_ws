cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)


# list of messages to generate
set(msg_files
  "msg/AddressBook.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(publish_address_book src/publish_address_book.cpp)
target_link_libraries(publish_address_book rclcpp::rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
  
target_link_libraries(publish_address_book "${cpp_typesupport_target}")

install(TARGETS
    publish_address_book
  DESTINATION lib/${PROJECT_NAME})

ament_package()
