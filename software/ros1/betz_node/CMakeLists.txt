cmake_minimum_required(VERSION 2.8.3)
project(betz_node)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  dynamic_reconfigure
)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

#find_library(QWT
#    NAMES qwt
#    REQUIRED)
#
#include_directories(qwt)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)

generate_dynamic_reconfigure_options(cfg/Betz.cfg)

add_service_files(FILES
  SetIntParam.srv
  SetFloatParam.srv
  GetIntParam.srv
  GetFloatParam.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    roslib
    sensor_msgs
    message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(betz
  bus.cpp
  drive.cpp
  lightweightserial.cpp
  packet/boot.cpp
  packet/crc.cpp
  packet/discovery.cpp
  packet/flash_read.cpp
  packet/flash_write.cpp
  packet/num_params.cpp
  packet/param_name_value.cpp
  packet/param_set_value.cpp
  packet/reset.cpp
  packet/packet.cpp
  param.cpp
  transport_serial.cpp
  transport_multicast.cpp
  uuid.cpp)

add_executable(betz_node
  betz_node.cpp
  cli_main.cpp
)

add_dependencies(betz_node
  betz_node_gencfg
  betz
  ${betz_node_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(betz_node betz ${catkin_LIBRARIES})


add_executable(betz_gui
  gui_main.cpp
  main_window.cpp
)
add_dependencies(betz_gui
  betz)
target_link_libraries(betz_gui
  betz
  Qt5::Widgets
  ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS betz_node betz_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
