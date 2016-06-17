cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(icp_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  roslib 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  pcl_ros
  image_transport 
  cv_bridge 
  tf
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters registration)
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
    roscpp 
    roslib 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    image_transport 
    cv_bridge 
    tf
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_library(
      ${PROJECT_NAME}_lib 
      ${PROJECT_SOURCE_DIR}/icp_pointclouds.cpp
      #${PROJECT_SOURCE_DIR}/mesh_sampling.cpp
)

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES})

add_executable(mesh_sampling mesh_sampling.cpp)
target_link_libraries(mesh_sampling ${PROJECT_NAME}_lib)

add_executable(icp_pointclouds icp_pointclouds.cpp)
target_link_libraries(icp_pointclouds ${PROJECT_NAME}_lib)

