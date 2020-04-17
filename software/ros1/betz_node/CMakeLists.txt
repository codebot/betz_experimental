cmake_minimum_required(VERSION 2.8.3)
project(betz_node)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  dynamic_reconfigure
)

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

add_executable(betz_node
  betz_node.cpp
  bus.cpp
  drive.cpp
  lightweightserial.cpp
  main.cpp
  packet/crc.cpp
  packet/discovery.cpp
  packet/num_params.cpp
  packet/packet.cpp
  transport_serial.cpp
  transport_multicast.cpp)

add_dependencies(betz_node
  betz_node_gencfg
  ${betz_node_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(betz_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS betz_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
