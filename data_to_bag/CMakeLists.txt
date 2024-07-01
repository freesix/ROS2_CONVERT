cmake_minimum_required(VERSION 3.20)
project(data_to_bag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(data_to_bag SHARED
  src/data_to_bag.cpp)

rclcpp_components_register_node(data_to_bag
  PLUGIN "DataToBag"
  EXECUTABLE data_to_bag_node)

ament_auto_package(INSTALL_TO_SHARE
  launch)
