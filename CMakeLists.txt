cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(icp_calibration)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters registration)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable(mesh_sampling mesh_sampling.cpp)
target_link_libraries(mesh_sampling ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES})

add_executable(icp_pointclouds icp_pointclouds.cpp)
target_link_libraries(icp_pointclouds ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} )