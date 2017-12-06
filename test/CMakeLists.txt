cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcd_recognition)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(include)



add_executable(pcd_recognition src/pcd_recognition.cpp)
add_executable(pcd_ply src/pcd_ply.cpp)
add_executable(compute_rsd src/compute_rsd.cpp)

target_link_libraries (pcd_recognition ${PCL_LIBRARIES})
target_link_libraries (pcd_ply ${PCL_LIBRARIES})
target_link_libraries (compute_rsd ${PCL_LIBRARIES})

