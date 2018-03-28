cmake_minimum_required(VERSION 2.8.3)
project(localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)
find_package(PCL 1.2 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (localization_node src/localization_node.cpp)
target_link_libraries (localization_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})


