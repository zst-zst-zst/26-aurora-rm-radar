cmake_minimum_required(VERSION 3.8)
project(localization LANGUAGES CUDA C CXX)
set(CMAKE_BUILD_TYPE "RELEASE") 

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

# VTK targets reference MPI::MPI_C → find MPI before VTK/PCL loads
find_package(MPI REQUIRED)

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()

# CUDA for GPU-accelerated ICP
find_package(CUDAToolkit REQUIRED)

set(LOCALIZATION_NODE "localization")
file(GLOB_RECURSE LOCALIZATION_NODE_SRCS "src/*.cpp")

# Build CUDA kernel as a separate static library (compiled by nvcc)
# The .cu file must NOT include Eigen/PCL headers
add_library(icp_kernel STATIC src/cuda_icp_kernel.cu)
set_target_properties(icp_kernel PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CUDA_ARCHITECTURES "89"
    LANGUAGE CUDA
)
target_include_directories(icp_kernel PUBLIC include)
target_link_libraries(icp_kernel PUBLIC CUDA::cudart)

# Main library (C++ files compiled by g++)
ament_auto_add_library(${LOCALIZATION_NODE} SHARED
    ${LOCALIZATION_NODE_SRCS}
)

target_link_libraries(${LOCALIZATION_NODE}
    ${PCL_LIBRARIES}
    icp_kernel
    CUDA::cudart
)

rclcpp_components_register_node(${LOCALIZATION_NODE}
    PLUGIN "tdt_radar::Localization"
    EXECUTABLE ${LOCALIZATION_NODE}_node
)

ament_auto_package ()
