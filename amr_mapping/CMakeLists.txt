cmake_minimum_required(VERSION 3.0.2)
project(amr_mapping)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    urg_node
    nav_msgs
    openslam_gmapping
)

catkin_package(

)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")
