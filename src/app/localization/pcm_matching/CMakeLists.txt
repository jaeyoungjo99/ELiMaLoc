cmake_minimum_required(VERSION 2.8.3)
project(pcm_matching)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    tf
    pcl_conversions
    pcl_ros
    cv_bridge
    std_msgs
    sensor_msgs
    geometry_msgs
    # autohyu_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(TBB REQUIRED) 
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}

    # BSW
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser

    ../localization_interface
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/trajectory.cpp
# )

add_executable(${PROJECT_NAME} 
    # BSW
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser/ini_parser.cpp
    
    src/voxel_hash_map.cpp
    src/registration.cpp
    src/pcm_matching.cpp
)
add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    TBB::tbb
)