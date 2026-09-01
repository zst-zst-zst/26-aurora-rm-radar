cmake_minimum_required(VERSION 3.8)
project(tdt_vision)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()
if(POLICY CMP0104)
    cmake_policy(SET CMP0104 NEW)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(  
  "detect/include"
  "resolve/include"
  "calibrate/include"
    "camera/include"
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
if(NOT CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES native)
endif()
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

add_subdirectory(utils)
include_directories(${PROJECT_SOURCE_DIR}/utils/include)
# utils is a plain CMake lib (not ament) so propagate vision_interface includes
target_include_directories(utils PUBLIC ${vision_interface_INCLUDE_DIRS})

set(RADAR_DETECT "radar_detect")
file(GLOB_RECURSE RADAR_DETECT_SRCS "detect/src/*.cpp")
ament_auto_add_library(${RADAR_DETECT} SHARED
    ${RADAR_DETECT_SRCS}
)
find_package(CUDA)
include(${PROJECT_SOURCE_DIR}/cmake/modules/libnvinferConfig.cmake)
message("Using Cuda Version: ${CUDA_VERSION} and TensorRT Version: ${nvinfer_VERSION}")
add_definitions(-D use_tensorrt)
enable_language(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${nvinfer_INCLUDE_DIRS})

add_subdirectory(infer)
include_directories(${PROJECT_SOURCE_DIR}/infer/include)

# ── Hikvision MVS SDK (optional) ─────────────────────────────────────────────
# 寻找顺序：
#   1) src/tdt_vision/camera/MV_SDK/x86_64  (项目本地，优先，自包含)
#   2) ENV MVS_SDK_PATH
#   3) /opt/MVS  (系统安装，最后备用)
set(HIK_SDK_PATH "")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/camera/MV_SDK/x86_64/include/MvCameraControl.h")
    set(HIK_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/camera/MV_SDK/x86_64")
elseif(DEFINED ENV{MVS_SDK_PATH} AND EXISTS "$ENV{MVS_SDK_PATH}/include/MvCameraControl.h")
    set(HIK_SDK_PATH "$ENV{MVS_SDK_PATH}")
elseif(EXISTS "/opt/MVS/include/MvCameraControl.h")
    set(HIK_SDK_PATH "/opt/MVS")
endif()

set(HIK_SDK_AVAILABLE FALSE)
if(HIK_SDK_PATH)
    # 寻找 libMvCameraControl.so：可能在 lib/ 或 lib/64/
    find_library(HIK_MV_LIB MvCameraControl
        HINTS "${HIK_SDK_PATH}/lib/64" "${HIK_SDK_PATH}/lib" "${HIK_SDK_PATH}/lib/x86_64"
        NO_DEFAULT_PATH)
    if(HIK_MV_LIB)
        set(HIK_SDK_AVAILABLE TRUE)
        message(STATUS "Using Hikvision MVS SDK: ${HIK_SDK_PATH}  lib=${HIK_MV_LIB}")
        include_directories("${HIK_SDK_PATH}/include")
        add_definitions(-DHAS_HIK_SDK)
    else()
        message(WARNING "MVS headers found but libMvCameraControl.so missing under ${HIK_SDK_PATH}")
    endif()
else()
    message(STATUS "Hikvision MVS SDK not found — HikBackend will be a runtime stub. "
                   "Install MVS dev SDK to /opt/MVS or set MVS_SDK_PATH to enable.")
endif()

target_link_libraries(${RADAR_DETECT}
    utils
    infer 
    ${nvinfer_LIBRARY} 
    cudart 
    nvonnxparser 
    fmt 
    ${PCL_LIBRARIES}
    )
rclcpp_components_register_node(${RADAR_DETECT}
    PLUGIN "tdt_radar::Detect"
    EXECUTABLE ${RADAR_DETECT}_node
)
rclcpp_components_register_node(${RADAR_DETECT}
    PLUGIN "tdt_radar::DetectDebugView"
    EXECUTABLE detect_debug_view_node
)

set(RADAR_RESOLVE "radar_resolve")
file(GLOB_RECURSE RADAR_RESOLVE_SRCS "resolve/src/*.cpp")

ament_auto_add_library(${RADAR_RESOLVE} SHARED
    ${RADAR_RESOLVE_SRCS}
)
rclcpp_components_register_node(${RADAR_RESOLVE}
    PLUGIN "tdt_radar::Resolve"
    EXECUTABLE ${RADAR_RESOLVE}_node
)

set(RADAR_CALIBRATE "radar_calibrate")
file(GLOB_RECURSE RADAR_CALIBRATE_SRCS "calibrate/src/*.cpp")
ament_auto_add_library(${RADAR_CALIBRATE} SHARED
    ${RADAR_CALIBRATE_SRCS}
)
rclcpp_components_register_node(${RADAR_CALIBRATE}
    PLUGIN "tdt_radar::Calibrate"
    EXECUTABLE ${RADAR_CALIBRATE}_node
)

set(RADAR_CAMERA "radar_camera")
file(GLOB_RECURSE RADAR_CAMERA_SRCS "camera/src/*.cpp")
if(RADAR_CAMERA_SRCS)
    ament_auto_add_library(${RADAR_CAMERA} SHARED
        ${RADAR_CAMERA_SRCS}
    )
    target_link_libraries(${RADAR_CAMERA} ${cv_bridge_LIBRARIES})
    if(HIK_SDK_AVAILABLE)
        target_link_libraries(${RADAR_CAMERA} ${HIK_MV_LIB})
        # 运行时能在 install/lib 旁边找到 libMvCameraControl.so
        set_target_properties(${RADAR_CAMERA} PROPERTIES
            INSTALL_RPATH "$ORIGIN"
            BUILD_RPATH_USE_ORIGIN TRUE)
        # 把捆绑的 .so 安装到包的 lib 目录
        get_filename_component(_hik_lib_dir "${HIK_MV_LIB}" DIRECTORY)
        file(GLOB _hik_so_files "${_hik_lib_dir}/libMv*.so*")
        install(FILES ${_hik_so_files} DESTINATION lib)
    endif()
    ament_target_dependencies(${RADAR_CAMERA} cv_bridge)
    rclcpp_components_register_node(${RADAR_CAMERA}
        PLUGIN "tdt_vision::NodeCamera"
        EXECUTABLE ${RADAR_CAMERA}_node
    )
endif()


ament_auto_package(
    INSTALL_TO_SHARE
    launch
)
