cmake_minimum_required(VERSION 2.8.3)
project(ekf_localization)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    tf
    cv_bridge
    std_msgs
    visualization_msgs
    geometry_msgs
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(GeographicLib REQUIRED)

catkin_package(
    INCLUDE_DIRS include
#  INCLUDE_DIRS include
#  LIBRARIES trajectory
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# add_subdirectory(${CMAKE_SOURCE_DIR}/lib/third_party/GeographicLib ${CMAKE_BINARY_DIR}/GeographicLib_build)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/third_party/InertialPoseLib ${CMAKE_BINARY_DIR}/InertialPoseLib_build)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GeographicLib_INCLUDE_DIRS}

    # BSW
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser

    # Library
    ../localization_interface
    # /usr/include/GeographicLib
    ${CMAKE_SOURCE_DIR}/lib/third_party/InertialPoseLib/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/trajectory.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} 
    # BSW
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser/ini_parser.cpp

    src/ekf_localization.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    libInertialPose
)