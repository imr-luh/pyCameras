# Install script for directory: /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util

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
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal_util.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal_util.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal_util.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/util/libmmal_util.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal_util.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal_util.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal_util.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/interface/mmal/util" TYPE FILE FILES
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_component_wrapper.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_connection.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_default_components.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_graph.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_il.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_list.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_param_convert.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_util.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_util_params.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/util/mmal_util_rational.h"
    )
endif()

