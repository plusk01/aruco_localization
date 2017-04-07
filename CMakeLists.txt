cmake_minimum_required(VERSION 2.8.3)
project(aruco_localization)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    cv_bridge
    sensor_msgs
    nav_msgs
    message_generation
    tf
    image_geometry
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

# aruco installs `Findaruco.cmake` to /usr/local/lib/cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake/")
find_package(aruco REQUIRED)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${aruco_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(aruco_localization src/aruco_localization_node.cpp src/aruco_localization/ArucoLocalizer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(aruco_localization ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS} stdc++fs)