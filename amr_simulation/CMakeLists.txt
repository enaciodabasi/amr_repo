cmake_minimum_required(VERSION 3.0.2)
project(amr_simulation)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)
find_package(gazebo REQUIRED)

catkin_package(
    DEPENDS
    gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS
)

include_directories(

)
