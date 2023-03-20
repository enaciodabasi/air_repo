cmake_minimum_required(VERSION 3.0.2)
project(amr_hardware_interface)

set(CMAKE_CXX_STANDARD 17)

set(AdsLib_INCLUDE_DIRS /home/$ENV{USER}/ADS/AdsLib/) 

set(ethercat_interface_lib /usr/local/lib/libethercat_interface.so)
set(ethercat_interface_lib_INCLUDE_DIRS /home/$ENV{USER}/Projects/igh_master/include/)

find_package(catkin REQUIRED COMPONENTS 
    soem 
    roscpp
    hardware_interface
    amr_custom_interfaces
    geometry_msgs
    nav_msgs
    sensor_msgs
    controller_manager
)

find_library(${ethercat_interface_lib} REQUIRED)


catkin_package(
    INCLUDE_DIRS
    include
    /home/$ENV{USER}/ADS/AdsLib/

    CATKIN_DEPENDS

    DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ethercat_interface_lib_INCLUDE_DIRS}
)



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

add_executable(amr_hw_ethercat 
    src/amr_hardware.cpp
)
target_link_libraries(amr_hw_ethercat ${catkin_LIBRARIES} ${ethercat_interface_lib} /usr/local/lib/libethercat.so)
target_link_directories(amr_hw_ethercat PUBLIC ${catkin_INCLUDE_DIRS} ${ethercat_interface_lib_INCLUDE_DIRS})
