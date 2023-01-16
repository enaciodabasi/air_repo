cmake_minimum_required(VERSION 3.0.2)
project(air_gui)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

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
    include/main_window.hpp
    include/central_widget.hpp
    include/axis_widget.hpp
    include/comms_handler.hpp
)
target_link_libraries(
    air_gui
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
)
target_link_directories(
    air_gui
    PUBLIC
    ${catkin_INCLUDE_DIRS}
)