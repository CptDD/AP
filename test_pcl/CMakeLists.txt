cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcd_reco)

find_package(PCL 1.8 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

find_package(PkgConfig REQUIRED) 
pkg_check_modules(FLANN REQUIRED flann) 

  
 link_directories(${FLANN_LIBRARY_DIRS}) 
 include_directories(${FLANN_INCLUDE_DIRS}) 
 if(FLANN_CFLAGS) 
   add_definitions(${FLANN_CFLAGS}) 
 endif(FLANN_CFLAGS)

SET(CMAKE_C_COMPILER mpicc)
SET(CMAKE_CXX_COMPILER mpicxx)
include_directories(MPI_INCLUDE_PATH)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_go src/go.cpp)
add_executable(build_tree src/build_tree.cpp)
add_executable(nearest_neighbors src/nearest_neighbors.cpp)
add_executable(compute_vfh src/compute_vfh.cpp)
add_executable(compute_rsd src/compute_rsd.cpp)

target_link_libraries (pcd_go ${PCL_LIBRARIES})
target_link_libraries (build_tree ${PCL_LIBRARIES} ${FLANN_LIBRARIES})
target_link_libraries (nearest_neighbors ${PCL_LIBRARIES} ${FLANN_LIBRARIES})
target_link_libraries (compute_vfh ${PCL_LIBRARIES} ${FLANN_LIBRARIES})
target_link_libraries (compute_rsd ${PCL_LIBRARIES} ${FLANN_LIBRARIES})