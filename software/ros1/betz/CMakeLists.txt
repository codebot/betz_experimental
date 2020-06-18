cmake_minimum_required(VERSION 2.8.3)
project(betz)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES betz
  CATKIN_DEPENDS
    roslib
    sensor_msgs
    message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(betz
  bus.cpp
  drive.cpp
  lightweightserial.cpp
  param.cpp
  transport_serial.cpp
  transport_multicast.cpp
  uuid.cpp
  packet/boot.cpp
  packet/crc.cpp
  packet/discovery.cpp
  packet/flash_read.cpp
  packet/flash_write.cpp
  packet/num_params.cpp
  packet/param_name_value.cpp
  packet/param_set_value.cpp
  packet/param_write_flash.cpp
  packet/reset.cpp
  packet/packet.cpp
  packet/state_poll.cpp)

add_executable(betz_node
  betz_node.cpp
  cli_main.cpp
)

add_dependencies(betz_node betz)
target_link_libraries(betz_node betz ${catkin_LIBRARIES})

add_executable(betz_gui
  gui_main.cpp
  main_window.cpp
)

add_dependencies(betz_gui
  betz)

target_include_directories(betz_gui
  PUBLIC /usr/include/qwt)

target_link_libraries(betz_gui
  betz
  Qt5::Widgets
  qwt-qt5
  ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS betz betz_node betz_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/betz/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
