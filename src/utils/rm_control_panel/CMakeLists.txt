cmake_minimum_required(VERSION 3.10)
project(rm_control_panel_cpp)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui Concurrent)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_interface REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

add_executable(${PROJECT_NAME}
  src/GUI.cpp
  src/RmControlPanel.cpp
  src/FieldMapWidget.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::Concurrent
  yaml-cpp::yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp vision_interface)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
