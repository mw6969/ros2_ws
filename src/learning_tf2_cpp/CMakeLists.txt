cmake_minimum_required(VERSION 3.8)
project(learning_tf2_cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtlesim_msgs REQUIRED)

add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
target_link_libraries(
   static_turtle_tf2_broadcaster PUBLIC
   ${geometry_msgs_TARGETS}
   rclcpp::rclcpp
   tf2::tf2
   tf2_ros::tf2_ros
)

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
target_link_libraries(
   turtle_tf2_broadcaster PUBLIC
   ${geometry_msgs_TARGETS}
   rclcpp::rclcpp
   tf2::tf2
   tf2_ros::tf2_ros
   ${turtlesim_msgs_TARGETS}
)

add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
target_link_libraries(
   turtle_tf2_listener PUBLIC
   ${geometry_msgs_TARGETS}
   rclcpp::rclcpp
   tf2::tf2
   tf2_ros::tf2_ros
   ${turtlesim_msgs_TARGETS}
)

add_executable(fixed_frame_tf2_broadcaster src/fixed_frame_tf2_broadcaster.cpp)
target_link_libraries(
   fixed_frame_tf2_broadcaster PUBLIC
   ${geometry_msgs_TARGETS}
   rclcpp::rclcpp
   tf2_ros::tf2_ros
)

add_executable(dynamic_frame_tf2_broadcaster src/dynamic_frame_tf2_broadcaster.cpp)
target_link_libraries(
   dynamic_frame_tf2_broadcaster PUBLIC
   ${geometry_msgs_TARGETS}
   rclcpp::rclcpp
   tf2_ros::tf2_ros
)

install(TARGETS
   static_turtle_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
   turtle_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
   turtle_tf2_listener
   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
   fixed_frame_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
   dynamic_frame_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME})

ament_package()
