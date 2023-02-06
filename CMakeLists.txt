cmake_minimum_required(VERSION 3.10)

project(ZWO_ASI_PYTHON_SDK LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# pybind11
add_subdirectory(3rdparty/pybind11)
include_directories(3rdparty/pybind11/include)

# ASI SDK
include_directories(ASI_SDK/include)
link_directories(ASI_SDK/lib/x64)

# ASICamera
add_subdirectory(ASICamera)