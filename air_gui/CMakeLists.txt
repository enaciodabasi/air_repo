cmake_minimum_required(VERSION 3.0.2)
project(air_gui)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    geometry_msgs
)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

catkin_package(
    INCLUDE_DIRS
    include

    CATKIN_DEPENDS

    DEPENDS


)

include_directories(
    ${catkin_INCLUDE_DIRS}
)



add_executable(air_gui
    src/main.cpp
    src/main_window.cpp
    src/central_widget.cpp
    src/axis_widget.cpp
    src/comms_handler.cpp
    src/start_stop_widget.cpp
    src/manual_control_widget.cpp
    src/rviz_widget.cpp
    src/system_diagnostics_widget.cpp
    include/main_window.hpp
    include/central_widget.hpp
    include/axis_widget.hpp
    include/comms_handler.hpp
    include/start_stop_widget.hpp
    include/manual_control_widget.hpp
    include/rviz_widget.hpp
    include/system_diagnostics_widget.hpp
)
target_link_libraries(
    air_gui
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    ${YAML_CPP_LIBRARIES}
)
target_link_directories(
    air_gui
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)