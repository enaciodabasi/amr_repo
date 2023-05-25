cmake_minimum_required(VERSION 3.0.2)
project(amr_custom_interfaces)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    message_generation
)

add_message_files(
    FILES
    EncoderData.msg
    WheelVelCmd.msg
)

add_service_files(
    FILES
    MapNavigationGoal.srv    
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
)

catkin_package(

)



include_directories(

)