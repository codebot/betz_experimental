cmake_minimum_required(VERSION 3.0.0)
project(betzdrive_emulator)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting CMAKE_BUILD_TYPE to Debug explicitly")
  set(CMAKE_BUILD_TYPE Debug)
endif()

include_directories(../common)

add_executable(comms_emulator
  main.c
  ../common/comms.c
  ../common/param.c
  emulator_uuid.c
  emulator_flash.c
  mcast_udp.c)
