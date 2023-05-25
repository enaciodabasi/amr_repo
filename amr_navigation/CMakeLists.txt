cmake_minimum_required(VERSION 3.0.2)
project(amr_navigation)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    move_base_msgs
    amr_custom_interfaces
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES amr_navigation
 CATKIN_DEPENDS roscpp actionlib move_base_msgs amr_custom_interfaces
)


include_directories(
    include
)

add_executable(
    map_goal_client
    src/map_goal.cpp
)
target_link_libraries(
    map_goal_client
    ${catkin_LIBRARIES}
)
target_include_directories(
    map_goal_client
    PUBLIC
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    base_goal_client
    src/base_goal.cpp
)
target_link_libraries(
    base_goal_client
    ${catkin_LIBRARIES}
)
target_include_directories(
    base_goal_client
    PUBLIC
    ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")
install(DIRECTORY params/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
    FILES_MATCHING PATTERN "*.yaml")