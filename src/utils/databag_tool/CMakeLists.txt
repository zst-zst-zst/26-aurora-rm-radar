cmake_minimum_required(VERSION 3.8)
project(databag_tool)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(DATABAG_TOOL databag_tool)
ament_auto_add_library(${DATABAG_TOOL} SHARED
  bag_recorder_node.cpp
)

rclcpp_components_register_node(${DATABAG_TOOL}
  PLUGIN "BagRecorderNode"
  EXECUTABLE ${DATABAG_TOOL}_node
)

ament_auto_package()
