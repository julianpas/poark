CMAKE_MINIMUM_REQUIRED(VERSION 2.4.6)
INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

SET(ARDUINO_BOARD mega2560 CACHE STRING "Board type to compile firmware for." )
SET(ARDUINO_PORT  /dev/ttyACM0 CACHE STRING "Port to use to communicate with Arduino board.")

ROSBUILD_FIND_ROS_PACKAGE(rosserial_arduino)
INCLUDE(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

MESSAGE("${rosserial_arduino_PACKAGE_PATH}")

SET(FIRMWARE_NAME poark_server)

SET(${FIRMWARE_NAME}_BOARD ${ARDUINO_BOARD})  # Arduino Target board
SET(${FIRMWARE_NAME}_SRCS poark_server.cpp)
SET(${FIRMWARE_NAME}_PORT ${ARDUINO_PORT})  # Serial upload port
GENERATE_ROS_FIRMWARE(${FIRMWARE_NAME})

MESSAGE(STATUS "Compiling firmware for:  ${ARDUINO_BOARD}")
MESSAGE(STATUS "Communication port:      ${ARDUINO_PORT}")
