cmake_minimum_required(VERSION 3.0.2)
project(cam_lidar_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Debug")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  tf
  tf_conversions
  yolo_ros
) 
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
MESSAGE(STATUS "OpenCV version: "${OpenCV_VERSION})
## Generate messages in the 'msg' folder
# add_message_files( 
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv 
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})

catkin_package()

add_executable(image_publisher utils/image_publisher.cpp)
target_link_libraries(image_publisher 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES})
add_executable(lidar_publisher utils/lidar_publisher.cpp)
target_link_libraries(lidar_publisher 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      yaml-cpp) 

add_executable(ground_filter src/ground_filter.cpp)
target_link_libraries(ground_filter 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}) 

add_executable(cloud_cluster src/cloud_cluster.cpp)
target_link_libraries(cloud_cluster 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES})   
add_executable(image_shower src/image_shower.cpp)
target_link_libraries(image_shower 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES})        