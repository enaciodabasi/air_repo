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
    AmrMoveGoalMsg.msg
)

add_service_files(
    FILES
    AmrMoveGoalSrv.srv
    OdomSrv.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_custom_interfaces
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)



include_directories(

)

