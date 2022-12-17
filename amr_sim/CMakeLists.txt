cmake_minimum_required(VERSION 3.0.2)
project(amr_sim)

set (CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    nav_msgs
    openslam_gmapping
    geometry_msgs
    move_base_msgs
    actionlib
    behaviortree_cpp_v3
    amr_custom_interfaces
)

#find_package(behaviortree_cpp_v3 REQUIRED)
#find_package(behaviortree_cpp_v3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES amr_localization behaviortree_cpp_v3
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp actionlib move_base_msgs behaviortree_cpp_v3 amr_custom_interfaces
    DEPENDS system_lib Boost 
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(amr_move_bt src/amr_sim_bt.cpp)
target_link_libraries(amr_move_bt ${catkin_LIBRARIES} Boost::thread)

add_executable(amr_move_client src/amr_move_client.cpp)
target_link_libraries(amr_move_client ${catkin_LIBRARIES})

install(DIRECTORY param DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)