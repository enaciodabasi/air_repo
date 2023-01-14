cmake_minimum_required(VERSION 3.0.2)
project(amr_control)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    diff_drive_controller
    hardware_interface
    roscpp
)

catkin_package(

)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)