cmake_minimum_required(VERSION 2.8.3)
project(velodyne16)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  tf
  message_generation 
  std_msgs
  pcl_ros
  pcl_conversions
  angles
)

find_package(Boost COMPONENTS signals)

# Use C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

add_message_files(
  FILES
  VelodynePacket.msg
)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
  config/Velodyne16Node.cfg config/CloudVLP16Node.cfg config/TransformVLP16Node.cfg
)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

catkin_package(
  INCLUDE_DIRS include
  DEPENDS yaml-cpp python-yaml
  CATKIN_DEPENDS roscpp dynamic_reconfigure message_runtime std_msgs tf pcl_ros angles
  LIBRARIES velodyne16_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(velodyne16_lib
src/velodyne16_lib/calibration.cc
src/velodyne16_lib/rawdata.cpp)

target_link_libraries(velodyne16_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(velodyne16_node src/velodyne16_node/velodyne16_node.cpp src/velodyne16_node/driver.cpp)
add_executable(cloud_vlp16_node src/cloud_vlp16_node/cloud_vlp16_node.cpp src/cloud_vlp16_node/convert.cpp)
add_executable(transform_vlp16_node src/transform_vlp16_node/transform_vlp16_node.cpp src/transform_vlp16_node/transform.cpp)

add_dependencies(velodyne16_node velodyne16_gencfg)
add_dependencies(cloud_vlp16_node velodyne16_gencfg)
add_dependencies(transform_vlp16_node velodyne16_gencfg)

target_link_libraries(velodyne16_node ${catkin_LIBRARIES})
target_link_libraries(cloud_vlp16_node velodyne16_lib ${catkin_LIBRARIES})
target_link_libraries(transform_vlp16_node velodyne16_lib ${catkin_LIBRARIES})

# Installation

install(TARGETS velodyne16_node cloud_vlp16_node velodyne16_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)