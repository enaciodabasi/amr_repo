cmake_minimum_required(VERSION 3.0.2)
project(amr_hardware)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    hardware_interface
    controller_manager
    realtime_tools
    std_msgs
    amr_custom_interfaces
)


catkin_package(
    INCLUDE_DIRS include

    CATKIN_DEPENDS roscpp hardware_interface realtime_tools controller_manager amr_custom_interfaces std_msgs
)


include_directories(
    include
)

add_executable(amr_hardware_interface
    src/amr_hardware_interface.cpp
)
target_link_libraries(amr_hardware_interface ${catkin_LIBRARIES})
target_include_directories(amr_hardware_interface PUBLIC ${catkin_INCLUDE_DIRS})

install(TARGETS amr_hardware_interface
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html



## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
