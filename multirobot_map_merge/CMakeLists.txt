cmake_minimum_required(VERSION 3.1)
project(multirobot_map_merge)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_geometry
  map_msgs
  nav_msgs
  roscpp
  tf2_geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

# OpenCV is required for merging without initial positions
find_package(OpenCV REQUIRED)
if("${OpenCV_VERSION}" VERSION_LESS "3.0")
  message(FATAL_ERROR "This package needs OpenCV >= 3.0")
endif()
if("${OpenCV_VERSION}" VERSION_LESS "4.0")
  message(WARNING "This package supports OpenCV 3, but some features may not be "
    "available. Upgrade to OpenCV 4 to take advantage of all features.")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################
# we don't have any

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    map_msgs
    nav_msgs
    roscpp
    tf2_geometry_msgs
  DEPENDS
    OpenCV
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

# we want static linking for now
add_library(combine_grid STATIC
  src/combine_grids/grid_compositor.cpp
  src/combine_grids/grid_warper.cpp
  src/combine_grids/merging_pipeline.cpp
)
add_dependencies(combine_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(combine_grid ${OpenCV_LIBRARIES})

add_executable(map_merge
  src/map_merge.cpp
)
add_dependencies(map_merge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merge combine_grid ${catkin_LIBRARIES})

#############
## Install ##
#############

# install nodes, installing combine_grids should not be necessary,
# but lets make catkin_lint happy
install(TARGETS combine_grid map_merge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
