cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_pkg)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g ")
find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package( )

include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(point_cloud_publisher pointpublisher.cpp)
target_link_libraries(point_cloud_publisher ${PCL_LIBRARIES}  ${catkin_LIBRARIES} )
