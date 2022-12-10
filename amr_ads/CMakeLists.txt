cmake_minimum_required(VERSION 3.0.2)
project(amr_ads)

find_package(catkin REQUIRED COMPONENTS rospy)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")