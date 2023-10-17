cmake_minimum_required(VERSION 2.8.3)
project(captoglove_ros_wrapper)

set(CMAKE_BUILD_TYPE Debug)

# Set CaptoGloveAPI (sources and library)
set(CAPTOGLOVEAPI_INCLUDE_DIR $ENV{CAPTOGLOVEAPI_SRC_PATH})
message(STATUS "CAPTOGLOVEAPI SRC PATH: " ${CAPTOGLOVEAPI_INCLUDE_DIR})
set(CAPTOGLOVEAPI_LIBRARY_DIR $ENV{CAPTOGLOVEAPI_LIBRARY_PATH})
message(STATUS "CAPTOGLOVEAPI LIBRARY PATH: " ${CAPTOGLOVEAPI_LIBRARY_DIR})

find_library(CAPTOGLOVEAPI_LIBRARY CaptoGloveAPI ${CAPTOGLOVEAPI_LIBRARY_DIR})
message (STATUS "CAPTOGLOVEAPI LIBRARY: " ${CAPTOGLOVEAPI_LIBRARY})
message (STATUS "CAPTOGLOVEAPI INCLUDE DIR:" ${CAPTOGLOVEAPI_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    cmake_modules
    std_msgs
    sensor_msgs
    message_generation
    qt_build)

find_package(Qt5 REQUIRED COMPONENTS
         SerialPort
         Core
         DBus
         Network
         Bluetooth)

message(STATUS "QT5: " ${Qt5_FOUND})
message(QT5_DIR "${Qt5_DIR}")

find_package(Protobuf REQUIRED)
message(STATUS "PROTOBUF LIBRARY: " ${PROTOBUF_LIBRARY})
message(STATUS "PROTOBUF INCLUDE DIR: " ${PROTOBUF_INCLUDE_DIR})

add_message_files(
        FILES
        FingerFeedbackMsg.msg
        BatteryLevelMsg.msg
        DeviceInformationMsg.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(    
    INCLUDE_DIRS src include msg ${CAPTOGLOVEAPI_INCLUDE_DIR}
    LIBRARIES ${CAPTOGLOVEAPI_LIBRARY} ${PROTOBUF_LIBRARY}
    CATKIN_DEPENDS message_runtime roscpp rospy std_msgs captoglove_ros_msgs
    DEPENDS)

include_directories(
    include
    msg
    ${catkin_INCLUDE_DIRS}
    ${CAPTOGLOVEAPI_INCLUDE_DIR}
    ${CAPTOGLOVEAPI_INCLUDE_DIR}/proto_impl
    ${PROTOBUF_LIBRARIES})

set(captoglove_ros_wrapper_SOURCES src/captoglove_ros.cpp src/ros_translate.cpp src/main.cpp)
set(captoglove_ros_wrapper_HEADERS include/captoglove_ros/captoglove_ros.h include/captoglove_ros/ros_translate.h)

add_definitions(-std=c++11)

QT5_WRAP_CPP(captoglove_ros_wrapper_SRC ${captoglove_ros_wrapper_HEADERS})

add_library(captoglove_ros_wrapper src/captoglove_ros.cpp src/ros_translate.cpp ${captoglove_ros_wrapper_SRC})

target_link_libraries(
    captoglove_ros_wrapper
    ${catkin_LIBRARIES}   
    ${CAPTOGLOVEAPI_LIBRARY})

qt5_use_modules(captoglove_ros_wrapper Core Bluetooth Network)

add_executable(captoglove_ros_wrapper_node src/main.cpp ${captoglove_ros_wrapper_SRC})

target_link_libraries(captoglove_ros_wrapper_node
    captoglove_ros_wrapper
    ${catkin_LIBRARIES} 
    ${CAPTOGLOVEAPI_LIBRARY}
    ${PROTOBUF_LIBRARIES})

add_dependencies(captoglove_ros_wrapper_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

