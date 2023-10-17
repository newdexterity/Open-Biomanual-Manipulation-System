cmake_minimum_required(VERSION 3.0.2)
project(moa_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  roscpp
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES melfa_fake_controller
 CATKIN_DEPENDS hardware_interface roscpp std_msgs
 )
#  DEPENDS system_lib

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)


add_library(moa_driver_plugin
  src/hardware_interface.cpp
)
target_link_libraries(moa_driver_plugin ${catkin_LIBRARIES})
add_dependencies(moa_driver_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(moa_driver_node
  src/moa_driver_node.cpp
  src/hardware_interface.cpp
)
target_link_libraries(moa_driver_node ${catkin_LIBRARIES})
add_dependencies(moa_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(TARGETS moa_driver_plugin moa_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES hardware_interface_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)