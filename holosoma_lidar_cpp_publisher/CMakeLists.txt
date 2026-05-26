cmake_minimum_required(VERSION 3.8)
project(holosoma_lidar_cpp_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_path(ZMQ_INCLUDE_DIR NAMES zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)
if(NOT ZMQ_INCLUDE_DIR OR NOT ZMQ_LIBRARY)
  message(FATAL_ERROR "ZeroMQ development files not found. Install libzmq3-dev.")
endif()

add_executable(lidar_cpp_publisher src/lidar_cpp_publisher.cpp)
target_compile_features(lidar_cpp_publisher PRIVATE cxx_std_17)
target_include_directories(lidar_cpp_publisher PRIVATE ${ZMQ_INCLUDE_DIR})
target_link_libraries(lidar_cpp_publisher ${ZMQ_LIBRARY})
target_compile_definitions(lidar_cpp_publisher PRIVATE HOLOSOMA_HAS_LIVOX_ROS_DRIVER2=1)
ament_target_dependencies(lidar_cpp_publisher livox_ros_driver2 rclcpp sensor_msgs)

install(TARGETS lidar_cpp_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
