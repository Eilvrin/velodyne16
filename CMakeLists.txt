cmake_minimum_required(VERSION 2.8.3)
project(velodyne16)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  tf
  message_generation 
  std_msgs
)

# Use C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_message_files(
  FILES
  VelodynePacket.msg
)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
  config/Velodyne16Node.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp dynamic_reconfigure message_runtime std_msgs tf
  LIBRARIES velodyne16_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(velodyne16_lib
#)

#target_link_libraries(velodyne16_lib ${catkin_LIBRARIES})

add_executable(velodyne16_node src/velodyne16_node/velodyne16_node.cpp src/velodyne16_node/driver.cpp)

add_dependencies(velodyne16_node velodyne16_gencfg)

#target_link_libraries(velodyne16_node velodyne16_lib ${catkin_LIBRARIES})
target_link_libraries(velodyne16_node ${catkin_LIBRARIES})

# Installation

#install(TARGETS velodyne16_lib velodyne16_node 
install(TARGETS velodyne16_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
