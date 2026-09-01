cmake_minimum_required(VERSION 3.8)
project(kalman_filter)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()

include_directories(include)

set(KALMAN_FILTER "kalman_filter")
file(GLOB_RECURSE KALMAN_FILTER_SRCS
    "src/*.cpp"
)
# radar_serial_node.cpp owns its own main() and must NOT be linked into the
# composable-component shared library; otherwise the plugin .so carries a
# second main + static initializers that fight the container's rcl context
# at dlopen time, causing 'double free or corruption' on load.
list(REMOVE_ITEM KALMAN_FILTER_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/radar_serial_node.cpp")

ament_auto_add_library(${KALMAN_FILTER} SHARED
    ${KALMAN_FILTER_SRCS}
)
target_link_libraries(${KALMAN_FILTER} ${PCL_LIBRARIES} yaml-cpp)

ament_auto_add_executable(radar_serial_node src/radar_serial_node.cpp)

rclcpp_components_register_node(${KALMAN_FILTER}
    PLUGIN "tdt_radar::KalmanFilter"
    EXECUTABLE ${KALMAN_FILTER}_node
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)



ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)