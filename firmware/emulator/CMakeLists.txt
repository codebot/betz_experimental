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
  ../common/control.c
  ../common/param.c
  ../common/state.c
  emulator_flash.c
  emulator_rng.c
  emulator_sys.c
  emulator_systime.c
  emulator_uuid.c
  multicast.c)

target_compile_definitions(comms_emulator PUBLIC EMULATOR)
