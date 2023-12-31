# Install script for directory: /home/irsl/catkin_ws/src/allegro_with_sensor

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/irsl/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/irsl/catkin_ws/build/allegro_with_sensor/catkin_generated/installspace/allegro_with_sensor.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/allegro_with_sensor/cmake" TYPE FILE FILES
    "/home/irsl/catkin_ws/build/allegro_with_sensor/catkin_generated/installspace/allegro_with_sensorConfig.cmake"
    "/home/irsl/catkin_ws/build/allegro_with_sensor/catkin_generated/installspace/allegro_with_sensorConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/allegro_with_sensor" TYPE FILE FILES "/home/irsl/catkin_ws/src/allegro_with_sensor/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/grasp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/grasp")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/grasp"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/irsl/catkin_ws/build/allegro_with_sensor/bin/grasp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/irsl/catkin_ws/build/allegro_with_sensor/bin" TYPE EXECUTABLE FILES "/home/irsl/catkin_ws/devel/lib/allegro_with_sensor/grasp")
  if(EXISTS "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/grasp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/grasp")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/grasp"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/grasp")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/test_can_comm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/test_can_comm")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/test_can_comm"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/irsl/catkin_ws/build/allegro_with_sensor/bin/test_can_comm")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/irsl/catkin_ws/build/allegro_with_sensor/bin" TYPE EXECUTABLE FILES "/home/irsl/catkin_ws/devel/lib/allegro_with_sensor/test_can_comm")
  if(EXISTS "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/test_can_comm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/test_can_comm")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/test_can_comm"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/irsl/catkin_ws/build/allegro_with_sensor/bin/test_can_comm")
    endif()
  endif()
endif()

