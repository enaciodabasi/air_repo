cmake_minimum_required(VERSION 3.0.2)
project(amr_description)

find_package(catkin REQUIRED)
find_package(roslaunch REQUIRED)

catkin_package(

)

foreach(DIR config launch meshes urdf)
    install(DIRECTORY ${DIR}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${DIR}
    )
endforeach(DIR)