cmake_minimum_required(VERSION 3.0.2)
project(air_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    hardware_interface
    roscpp    
)
set(AdsLib_INCLUDE_DIRS /home/$ENV{USER}/ADS/AdsLib/) 
catkin_package(
    INCLUDE_DIRS
    include
    /home/$ENV{USER}/ADS/AdsLib/

    CATKIN_DEPENDS
    controller_manager
    hardware_interface
    air_arm_robot
    roscpp

    DEPENDS
    AdsLib
)

add_library(ADSLIB
${AdsLib_INCLUDE_DIRS}/standalone/AdsLib.cpp
${AdsLib_INCLUDE_DIRS}/standalone/AmsConnection.cpp
${AdsLib_INCLUDE_DIRS}/standalone/AmsNetId.cpp
${AdsLib_INCLUDE_DIRS}/standalone/AmsPort.cpp
${AdsLib_INCLUDE_DIRS}/standalone/AmsRouter.cpp
${AdsLib_INCLUDE_DIRS}/standalone/NotificationDispatcher.cpp
${AdsLib_INCLUDE_DIRS}/AdsDef.cpp
${AdsLib_INCLUDE_DIRS}/AdsDevice.cpp
${AdsLib_INCLUDE_DIRS}/AdsFile.cpp
${AdsLib_INCLUDE_DIRS}/Log.cpp
${AdsLib_INCLUDE_DIRS}/Sockets.cpp
${AdsLib_INCLUDE_DIRS}/Frame.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AdsLib_INCLUDE_DIRS}
)

add_executable(air_right_arm_hw_interface_node
    include/air_arm_hardware.hpp
    src/air_arm_hardware_interface.cpp
    src/air_right_arm_interface.cpp
    ${ads_SRC}
    )

target_link_libraries(air_right_arm_hw_interface_node ${catkin_LIBRARIES} ADSLIB)
target_link_directories(air_right_arm_hw_interface_node PUBLIC ${AdsLib_INCLUDE_DIRS})
