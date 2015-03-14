cmake_minimum_required(VERSION 2.8.3)
project(bundler_parser)

find_package(catkin_simple REQUIRED)
find_package(opencv2_catkin REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++0x -Wall -Wextra -pedantic)

cs_add_library(${PROJECT_NAME} src/bundler-parser.cc)

cs_add_executable(parse_bundler src/test-bundler-parser.cc)

target_link_libraries(parse_bundler ${PROJECT_NAME} ${catkin_LIBRARIES})

cs_install()
cs_export() 
