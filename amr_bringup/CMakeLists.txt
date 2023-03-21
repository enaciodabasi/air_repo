cmake_minimum_required(VERSION 3.0.2)
project(amr_bringup)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    diff_drive_controller
)


catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    roscpp
    controller_manager
    diff_drive_controller
    class_loader
    DEPENDS
)


include_directories(

)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)