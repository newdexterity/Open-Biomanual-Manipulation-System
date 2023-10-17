cmake_minimum_required(VERSION 2.8.3)
project(ndx_util)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  cv_bridge
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros std_msgs message_runtime
  DEPENDS EIGEN3
#  INCLUDE_DIRS include
#  LIBRARIES ndx_util
#  DEPENDS system_lib
)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Library
add_library(ndx_util
  src/tf_util.cpp
)

target_link_libraries(ndx_util ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# end

add_executable(test_ndx_utils src/tests.cpp src/tf_util.cpp)
target_link_libraries(test_ndx_utils ${catkin_LIBRARIES})

add_executable(tf_router src/tf_router.cpp src/tf_util.cpp)
target_link_libraries(tf_router ${catkin_LIBRARIES})

add_executable(marker_mesh_node src/marker_mesh_node.cpp)
target_link_libraries(marker_mesh_node ${catkin_LIBRARIES})

add_executable(wrench_frame_changer_node src/wrench_frame_changer_node.cpp)
target_link_libraries(wrench_frame_changer_node ${catkin_LIBRARIES})