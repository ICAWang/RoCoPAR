cmake_minimum_required(VERSION 3.1)
project(rocopar_map_merge)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_geometry
  map_msgs
  nav_msgs
  roscpp
  tf2_geometry_msgs
  message_generation
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
## Generate services in the 'srv' folder
add_service_files(FILES
  MapMergeService.srv
)

###################################
## catkin specific configuration ##
###################################
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    map_msgs
    nav_msgs
    roscpp
    tf2_geometry_msgs
    message_runtime
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
add_library(combine_grids STATIC
  src/combine_grids/grid_compositor.cpp
  src/combine_grids/grid_warper.cpp
  src/combine_grids/merging_pipeline.cpp
)
add_dependencies(combine_grids ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(combine_grids ${OpenCV_LIBRARIES})

add_executable(map_merge_dist
  src/map_merge.cpp
)
add_dependencies(map_merge_dist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merge_dist combine_grids ${catkin_LIBRARIES})

#############
## Install ##
#############

# install nodes, installing combine_grids should not be necessary,
# but lets make catkin_lint happy
install(TARGETS combine_grids map_merge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # download test data
  set(base_url https://raw.githubusercontent.com/hrnr/m-explore-extra/master/map_merge)
  catkin_download_test_data(${PROJECT_NAME}_map00.pgm ${base_url}/hector_maps/map00.pgm MD5 915609a85793ec1375f310d44f2daf87)
  catkin_download_test_data(${PROJECT_NAME}_map05.pgm ${base_url}/hector_maps/map05.pgm MD5 cb9154c9fa3d97e5e992592daca9853a)
  catkin_download_test_data(${PROJECT_NAME}_2011-08-09-12-22-52.pgm ${base_url}/gmapping_maps/2011-08-09-12-22-52.pgm MD5 3c2c38e7dec2b7a67f41069ab58badaa)
  catkin_download_test_data(${PROJECT_NAME}_2012-01-28-11-12-01.pgm ${base_url}/gmapping_maps/2012-01-28-11-12-01.pgm MD5 681e704044889c95e47b0c3aadd81f1e)

  catkin_add_gtest(test_merging_pipeline test/test_merging_pipeline.cpp)
  # ensure that test data are downloaded before we run tests
  add_dependencies(test_merging_pipeline ${PROJECT_NAME}_map00.pgm ${PROJECT_NAME}_map05.pgm ${PROJECT_NAME}_2011-08-09-12-22-52.pgm ${PROJECT_NAME}_2012-01-28-11-12-01.pgm)
  target_link_libraries(test_merging_pipeline combine_grids ${catkin_LIBRARIES})

  # test all launch files
  # do not test from_map_server.launch as we don't want to add dependency on map_server and this
  # launchfile is not critical
  roslaunch_add_file_check(launch/map_merge.launch)
  roslaunch_add_file_check(launch/experiments)
endif()
