# Install script for directory: /home/wolfv/Programs/libpointmatcher

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/libpointmatcher" TYPE FILE FILES "/home/wolfv/Programs/libpointmatcher/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  foreach(file
      "$ENV{DESTDIR}/usr/local/lib/libpointmatcher.so.1.2.1"
      "$ENV{DESTDIR}/usr/local/lib/libpointmatcher.so.1"
      "$ENV{DESTDIR}/usr/local/lib/libpointmatcher.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libpointmatcher.so.1.2.1;/usr/local/lib/libpointmatcher.so.1;/usr/local/lib/libpointmatcher.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES
    "/home/wolfv/Programs/libpointmatcher/libpointmatcher.so.1.2.1"
    "/home/wolfv/Programs/libpointmatcher/libpointmatcher.so.1"
    "/home/wolfv/Programs/libpointmatcher/libpointmatcher.so"
    )
  foreach(file
      "$ENV{DESTDIR}/usr/local/lib/libpointmatcher.so.1.2.1"
      "$ENV{DESTDIR}/usr/local/lib/libpointmatcher.so.1"
      "$ENV{DESTDIR}/usr/local/lib/libpointmatcher.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/pointmatcher/PointMatcher.h;/usr/local/include/pointmatcher/PointMatcherPrivate.h;/usr/local/include/pointmatcher/Parametrizable.h;/usr/local/include/pointmatcher/Registrar.h;/usr/local/include/pointmatcher/Timer.h;/usr/local/include/pointmatcher/IO.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/pointmatcher" TYPE FILE FILES
    "/home/wolfv/Programs/libpointmatcher/pointmatcher/PointMatcher.h"
    "/home/wolfv/Programs/libpointmatcher/pointmatcher/PointMatcherPrivate.h"
    "/home/wolfv/Programs/libpointmatcher/pointmatcher/Parametrizable.h"
    "/home/wolfv/Programs/libpointmatcher/pointmatcher/Registrar.h"
    "/home/wolfv/Programs/libpointmatcher/pointmatcher/Timer.h"
    "/home/wolfv/Programs/libpointmatcher/pointmatcher/IO.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/libpointmatcher" TYPE FILE FILES "/home/wolfv/Programs/libpointmatcher/README.md")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/libpointmatcher/api" TYPE DIRECTORY FILES "/home/wolfv/Programs/libpointmatcher/doc/html")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/pointmatcher/libpointmatcherConfig.cmake;/usr/local/lib/cmake/pointmatcher/libpointmatcherConfigVersion.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/pointmatcher" TYPE FILE FILES
    "/home/wolfv/Programs/libpointmatcher/CMakeFiles/libpointmatcherConfig.cmake"
    "/home/wolfv/Programs/libpointmatcher/libpointmatcherConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/libpointmatcher/cmake" TYPE FILE FILES "/home/wolfv/Programs/libpointmatcher/libpointmatcherConfig.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/wolfv/Programs/libpointmatcher/contrib/cmake_install.cmake")
  include("/home/wolfv/Programs/libpointmatcher/examples/cmake_install.cmake")
  include("/home/wolfv/Programs/libpointmatcher/evaluations/cmake_install.cmake")
  include("/home/wolfv/Programs/libpointmatcher/utest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/wolfv/Programs/libpointmatcher/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
