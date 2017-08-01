# Install script for directory: /home/pi/libkindrv/src

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

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkindrv.so.0.1.2"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkindrv.so.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkindrv.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/pi/libkindrv/build/src/libkindrv.so.0.1.2"
    "/home/pi/libkindrv/build/src/libkindrv.so.0"
    "/home/pi/libkindrv/build/src/libkindrv.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkindrv.so.0.1.2"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkindrv.so.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkindrv.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/libkindrv" TYPE FILE FILES
    "/home/pi/libkindrv/include/Defined_Macro.h"
    "/home/pi/libkindrv/include/Robot_IMU_Control.h"
    "/home/pi/libkindrv/include/exception.h"
    "/home/pi/libkindrv/include/kindrv.h"
    "/home/pi/libkindrv/include/mySerial.h"
    "/home/pi/libkindrv/include/types.h"
    )
endif()

