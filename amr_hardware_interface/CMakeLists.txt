cmake_minimum_required(VERSION 3.0.2)
project(amr_hardware_interface)
set(AdsLib_INCLUDE_DIRS /home/$ENV{USER}/ADS/AdsLib/) 
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


catkin_package(
    INCLUDE_DIRS
    include
    /home/$ENV{USER}/ADS/AdsLib/

    CATKIN_DEPENDS

    DEPENDS
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



#add_executable(encoder_demo src/encoder_pub_demo.cpp)
#target_link_libraries(encoder_demo ${catkin_LIBRARIES})
#target_link_directories(encoder_demo PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(amr_hwi_node
    src/amr_hwi.cpp
    src/io_ads.cpp
)
target_link_libraries(amr_hwi_node ${catkin_LIBRARIES} ADSLIB)
target_link_directories(amr_hwi_node PUBLIC ${AdsLib_INCLUDE_DIRS})

add_executable(amr_controlled_hwi_node
    src/amr_ddc_hwi.cpp
    src/amr_hwi_utility.cpp
    src/io_ads.cpp
)
target_link_libraries(amr_controlled_hwi_node ${catkin_LIBRARIES} ADSLIB)
target_link_directories(amr_controlled_hwi_node PUBLIC ${AdsLib_INCLUDE_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)