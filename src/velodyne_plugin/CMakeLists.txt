cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#add_definitions(-DMODEL_PRINT_CUSTOM_INFO)
add_definitions(-DSENSOR_PRINT_CUSTOM_INFO)

# Build our plugin
add_library(velodyne_model_plugin SHARED velodyne_model_plugin.h velodyne_model_plugin.cpp)
target_link_libraries(velodyne_model_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(velodyne_sensor_plugin SHARED velodyne_sensor_plugin.h velodyne_sensor_plugin.cpp)
target_link_libraries(velodyne_sensor_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
