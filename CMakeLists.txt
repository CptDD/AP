cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcd_registration)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_executable(pcd_test ${SOURCES})


target_link_libraries (pcd_test ${PCL_LIBRARIES})