cmake_minimum_required(VERSION 3.0.2)
project(conversion_pkg)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(traductor_node src/traductor.cpp)
target_link_libraries(traductor_node ${catkin_LIBRARIES})

add_executable(tf_listener_node src/tf_listener.cpp)
target_link_libraries(tf_listener_node ${catkin_LIBRARIES})