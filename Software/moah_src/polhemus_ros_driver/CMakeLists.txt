cmake_minimum_required(VERSION 2.8.12)
project(polhemus_ros_driver)

add_compile_options(-std=c++11)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb libusb QUIET)

if(NOT ${libusb_FOUND})
  message(FATAL_ERROR "Could not find libusb-dev")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2
  tf2_ros
  roscpp
)

catkin_package(
  LIBRARIES polhemus_liberty
  CATKIN_DEPENDS geometry_msgs tf2 tf2_ros roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${libusb_INCLUDE_DIRS}
)

add_library(polhemus_liberty
  src/liberty.cpp
)

add_executable(polhemus_tf_broadcaster src/polhemus_tf_broadcaster.cpp)
target_link_libraries(polhemus_tf_broadcaster
  ${catkin_LIBRARIES}
  ${libusb_LIBRARIES}
  polhemus_liberty
)

install(TARGETS polhemus_tf_broadcaster polhemus_liberty
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/start.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
