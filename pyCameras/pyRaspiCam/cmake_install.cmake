# Install script for directory: /home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/libmmal.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal.so"
         OLD_RPATH "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/vc:/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/components:/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/core:/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/util:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmmal.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/interface/mmal" TYPE FILE FILES
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_buffer.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_clock.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_common.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_component.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_encodings.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_events.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_format.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_logging.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_metadata.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_parameters.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_parameters_audio.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_parameters_camera.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_parameters_clock.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_parameters_common.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_parameters_video.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_pool.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_port.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_queue.h"
    "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/interface/mmal/mmal_types.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/core/cmake_install.cmake")
  include("/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/util/cmake_install.cmake")
  include("/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/vc/cmake_install.cmake")
  include("/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/components/cmake_install.cmake")
  include("/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/openmaxil/cmake_install.cmake")
  include("/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/client/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/middendorf/Software/Python/pyCameras/pyCameras/pyRaspiCam/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
