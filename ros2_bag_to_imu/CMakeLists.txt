cmake_minimum_required(VERSION 3.20)
project(ros2_bag_to_imu)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)

find_package(Boost REQUIRED)

ament_auto_add_library(bag_to_imu SHARED 
  src/ros2_bag_to_imu.cpp)

rclcpp_components_register_node(bag_to_imu
  PLUGIN "BagToImu"
  EXECUTABLE bag_to_imu_node)

ament_auto_package(INSTALL_TO_SHARE
  launch)
