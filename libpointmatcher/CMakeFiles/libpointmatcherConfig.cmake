# - Config file for the libpointmatcher package
# It defines the following variables
#  libpointmatcher_INCLUDE_DIRS - include directories for pointmatcher
#  libpointmatcher_LIBRARIES    - libraries to link against
 
# Compute paths
get_filename_component(POINTMATCHER_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(libpointmatcher_INCLUDE_DIRS "/usr/local/include;/usr/include;/usr/include/eigen3;/home/wolfv/Programs/libpointmatcher/libnabo;/usr/include/eigen3;/usr/include")

set(libpointmatcher_LIBRARIES "/usr/local/lib/libpointmatcher.so;/usr/lib64/libboost_thread.so;/usr/lib64/libboost_filesystem.so;/usr/lib64/libboost_system.so;/usr/lib64/libboost_program_options.so;/usr/lib64/libboost_date_time.so;/usr/lib64/libboost_chrono.so;/home/wolfv/Programs/libpointmatcher/libnabo//libnabo.a;gomp;/home/wolfv/Programs/libpointmatcher/contrib/yaml-cpp-pm/libyaml-cpp-pm.a;rt")

# This causes catkin simple to link against these libraries
set(libpointmatcher_FOUND_CATKIN_PROJECT true)
