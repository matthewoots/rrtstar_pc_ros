cmake_minimum_required(VERSION 3.0.2)
project(rrtstar_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin 
  COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  common_msgs
  tf
  tf2_ros
  tf2_sensor_msgs
  pcl
  pcl_ros
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  point_array.msg
  bounding_box.msg
  bounding_box_array.msg
  start_end_markers.msg
)

# Generate added messages
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  roscpp 
  std_msgs 
  common_msgs
  tf
  tf2_ros
  pcl
  
  DEPENDS
  Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  external_packages/csv
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/task.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(${PROJECT_NAME}_node 
# src/main.cpp
# )

## All these are for testing purposes so we can leave them here
add_executable(${PROJECT_NAME}_test_node 
src/main.cpp
)
add_executable(${PROJECT_NAME}_visualization_node 
src/visualization/rrtstar_visualization.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

## All these are for testing purposes so we can leave them here
target_link_libraries(${PROJECT_NAME}_test_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_visualization_node 
  ${catkin_LIBRARIES}
)