cmake_minimum_required(VERSION 3.0.2)
project(gazebo_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  yolo_ros
)
find_package(Eigen3)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARIES_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  bbox.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  bboxPerFrame.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_plugin
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(MovingObj SHARED src/MovingObj.cc)
add_library(GetBBox SHARED src/GetBBox.cc)

set_target_properties(MovingObj PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(GetBBox PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

target_link_libraries(MovingObj ${GAZEBO_LIBRARIES})
target_link_libraries(GetBBox ${GAZEBO_LIBRARIES})

add_executable(test_detection src/test_detection.cpp)
target_link_libraries(test_detection ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES})