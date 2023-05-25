cmake_minimum_required(VERSION 3.0.2)
project(amr_gui)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rviz
    geometry_msgs
    sensor_msgs
    amr_custom_interfaces
)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
    INCLUDE_DIRS include/amr_gui
    LIBRARIES amr_gui
    CATKIN_DEPENDS roscpp rviz geometry_msgs sensor_msgs amr_custom_interfaces
)
set(include include/${PROJECT_NAME})

set(SOURCES
    src/main.cpp
    src/main_window.cpp
    ${include}/main_window.hpp
    src/ros_comm.cpp
    ${include}/ros_comm.hpp
    src/robot_info_widget.cpp
    ${include}/robot_info_widget.hpp
    ${include}/defs.hpp
    src/map_points_widget.cpp
    ${include}/map_points_widget.hpp
)
#include_directories(
#
#)

add_executable(
    ${PROJECT_NAME}
    ${SOURCES}
)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets Threads::Threads ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} include/)