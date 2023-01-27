cmake_minimum_required(VERSION 3.0.2)
project(air_arm_control)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
    moveit_task_constructor_core
    moveit_ros_planning_interface
    rosparam_shortcuts
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
