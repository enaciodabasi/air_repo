cmake_minimum_required(VERSION 3.0.2)
project(amr_bringup)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
)


catkin_package(

)


include_directories(

)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)