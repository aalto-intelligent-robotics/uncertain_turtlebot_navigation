cmake_minimum_required(VERSION 2.8.3)
project(uncertain_turtlebot_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  nav_msgs
  geometry_msgs
  roscpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(execute_navigation_goals src/execute_navigation_goals.cpp)
target_link_libraries(execute_navigation_goals ${catkin_LIBRARIES})

