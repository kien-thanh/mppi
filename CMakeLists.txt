cmake_minimum_required(VERSION 3.10)

# Project name and language
project(MyProject VERSION 1.0 LANGUAGES CXX CUDA)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/submodules/MPPI-Generic/cmake")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/submodules/MPPI-Generic)
include(MPPIGenericToolsConfig)

# Add include directory
include_directories(include src)

# Add the executable
add_executable(myprogram
    src/main.cu
    src/math_utils.cpp
)
target_link_libraries(myprogram MPPI::MPPI)
