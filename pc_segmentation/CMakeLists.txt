cmake_minimum_required(VERSION 2.8.3)
project(pc_segmentation)

find_package(catkin_simple REQUIRED)
find_package(opencv2_catkin REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

link_directories(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_definitions(--std=c++0x -Wall -Wextra -pedantic)

cs_add_library(${PROJECT_NAME} )

cs_add_executable(segment_pc )

target_link_libraries(segment_pc ${PROJECT_NAME} ${catkin_LIBRARIES})

cs_install()
cs_export() 
