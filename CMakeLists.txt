cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl-octomap)

find_package(PCL 1.4 REQUIRED)

find_package(OCTOMAP REQUIRED)

find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable (pcl-octomap pcl-octomap.cpp)
target_link_libraries (pcl-octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
