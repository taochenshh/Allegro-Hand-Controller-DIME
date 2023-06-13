cmake_minimum_required(VERSION 2.8.13)
project(allegro_hand)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(ll4ma_kdl_DIR "/root/catkin_ws/src/Allegro-Hand-Controller-DIME/src/ll4ma_kdl")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  allegro_hand_driver
  libpcan
  roscpp
  ll4ma_kdl
  sensor_msgs
#  bhand
# roslaunch  # Disabled due to testing
)

# catkin_python_setup()

add_subdirectory(thirdparty/pybind11)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES allegro_hand_controllers
  CATKIN_DEPENDS allegro_hand_driver libpcan pybind11_catkin roscpp sensor_msgs # ll4ma_kdl bhand 
#  DEPENDS
)
###########
## Build ##
###########
message( ${catkin_INCLUDE_DIRS} )
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare executables


# This is disabled until roslaunch tests can pass required arguments.
# roslaunch_add_file_check(launch/allegro_hand.launch)

#############
## Install ##
#############

install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  FILES_MATCHING PATTERN "*.launch"
)

# add_executable(${PROJECT_NAME}_node src/AllegroController.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# target_include_directories(${PROJECT_NAME}_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${catkin_INCLUDE_DIRS}
# )

add_library(${PROJECT_NAME} 
  src/AllegroController.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)

pybind11_add_module(AllegroControllerPy MODULE src/AllegroControllerBindings.cpp src/AllegroController.cpp)
target_link_libraries(AllegroControllerPy PRIVATE  ${PROJECT_NAME} ${catkin_LIBRARIES})

# install(TARGETS ${PROJECT_NAME}_node AllegroControllerPy
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

set_target_properties(AllegroControllerPy PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

# add_custom_command(TARGET AllegroControllerPy
#     POST_BUILD COMMAND
#     ${CMAKE_COMMAND} -E copy
#     $<TARGET_FILE:AllegroControllerPy>
#     ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/$<TARGET_FILE_NAME:AllegroControllerPy>
# )

