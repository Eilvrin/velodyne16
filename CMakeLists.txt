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
    nodelet
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
    config/DriverNode.cfg config/CloudNode.cfg
)

if (NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif (NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

catkin_package(
    INCLUDE_DIRS include
    DEPENDS yaml-cpp python-yaml
    CATKIN_DEPENDS roscpp dynamic_reconfigure message_runtime std_msgs tf pcl_ros angles nodelet
    LIBRARIES velodyne16_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(driver_node16 src/driver_node.cpp src/driver.cpp)
add_executable(cloud_node16 src/cloud_node.cpp src/calibration.cpp src/convert.cpp src/rawdata.cpp)
add_executable(transform_node16 src/transform_node.cpp src/transform.cpp)

add_dependencies(driver_node16 velodyne16_generate_messages_cpp velodyne16_gencfg)
add_dependencies(cloud_node16 velodyne16_generate_messages_cpp velodyne16_gencfg)

target_link_libraries(driver_node16 ${catkin_LIBRARIES})
target_link_libraries(cloud_node16 ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(transform_node16 ${catkin_LIBRARIES})

# Nodelet
add_library(driver_nodelet16 src/driver_nodelet.cpp src/driver.cpp)
add_dependencies(driver_nodelet16 velodyne16_generate_messages_cpp velodyne16_gencfg)
target_link_libraries(driver_nodelet ${catkin_LIBRARIES})

add_library(cloud_nodelet16 src/cloud_nodelet.cpp src/calibration.cpp src/convert.cpp src/rawdata.cpp)
add_dependencies(cloud_nodelet16 velodyne16_generate_messages_cpp velodyne16_gencfg)
target_link_libraries(cloud_nodelet16 ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(transform_nodelet16 src/transform_nodelet.cpp src/transform.cpp)
target_link_libraries(transform_nodelet16 ${catkin_LIBRARIES})

# Installation

install(TARGETS driver_node16 cloud_node16 transform_node16 driver_nodelet16 cloud_nodelet16 transform_nodelet16
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(FILES nodelet_velodyne16.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})