cmake_minimum_required(VERSION 3.0.2)
project(amr_localization)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    tf2
    amr_custom_interfaces
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES amr_localization
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
    DEPENDS system_lib
)

#add_executable(amr_wheel_odometry_node src/wheel_odom_node.cpp)
#target_link_libraries(amr_wheel_odometry_node ${catkin_LIBRARIES})

add_executable(amr_odom_server src/wheel_odom_server.cpp)
target_link_libraries(amr_odom_server ${catkin_LIBRARIES})
target_link_directories(amr_odom_server PUBLIC ${catkin_INCLUDE_DIRS})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)


# add_executable(${PROJECT_NAME}_node src/amr_localization_node.cpp)

