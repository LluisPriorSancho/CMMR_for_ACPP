cmake_minimum_required(VERSION 3.8)
project(spawner_robots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_export_dependencies(rclcpp)

install(
  DIRECTORY
    launch  
    config
    src
    srv
    rviz
  DESTINATION share/${PROJECT_NAME}
)
add_executable(robot_description_publisher src/robot_description_publisher.cpp)
ament_target_dependencies(robot_description_publisher rclcpp std_msgs)

add_executable(takeoff_node src/takeoff_node.cpp)
ament_target_dependencies(takeoff_node rclcpp tello_msgs)

add_executable(random_publisher src/random_publisher.cpp)
ament_target_dependencies(random_publisher rclcpp geometry_msgs)

add_executable(localization_node src/localization_node.cpp)
ament_target_dependencies(localization_node rclcpp gazebo_msgs tf2_ros geometry_msgs)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
  DEPENDENCIES rclcpp
 )
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(takeoff_node "${cpp_typesupport_target}")

install(TARGETS
  robot_description_publisher
  takeoff_node
  localization_node
  random_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
