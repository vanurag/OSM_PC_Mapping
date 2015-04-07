cmake_minimum_required(VERSION 2.8.3)
project(pc_segmentation)

find_package(catkin_simple REQUIRED)
find_package(opencv2_catkin REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(--std=c++0x -Wall -Wextra -pedantic)

cs_add_library(${PROJECT_NAME} src/flags.cc
                               src/pc-handler.cc)

cs_add_executable(segment_pc src/segment-pc.cc)

target_link_libraries(segment_pc ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

cs_install()
cs_export() 
