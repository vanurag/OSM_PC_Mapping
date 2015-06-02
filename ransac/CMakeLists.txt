cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(random_sample_consensus)

find_package(PCL 1.2 REQUIRED)

find_package(VTK 6.1 REQUIRED NO_MODULE) 
include(${VTK_USE_FILE}) 

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (random_sample_consensus test.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES} ${VTK_LIBRARIES} )

add_executable (filters filters.cpp)
target_link_libraries (filters ${PCL_LIBRARIES} ${VTK_LIBRARIES} )

add_executable (ransac ransac.cpp)
target_link_libraries (ransac ${PCL_LIBRARIES} ${VTK_LIBRARIES} )
