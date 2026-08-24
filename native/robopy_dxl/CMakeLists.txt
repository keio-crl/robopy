cmake_minimum_required(VERSION 3.20)
project(robopy_dxl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(ROBOPY_DXL_SDK_TAG "4.0.5" CACHE STRING
    "DynamixelSDK git tag to build against when no local checkout is given")
set(ROBOPY_DXL_SDK_DIR "" CACHE PATH
    "Path to an existing DynamixelSDK checkout (skips the download)")

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

# --- DynamixelSDK (C++) -------------------------------------------------------
# The SDK ships a CMakeLists.txt that installs a shared library system wide.
# We only want the sources, statically linked into the extension, so the wheel
# stays self contained.  Pointing SOURCE_SUBDIR at a directory without a
# CMakeLists.txt makes FetchContent download without calling add_subdirectory().
if(ROBOPY_DXL_SDK_DIR)
  set(dynamixel_sdk_SOURCE_DIR "${ROBOPY_DXL_SDK_DIR}")
  message(STATUS "Using DynamixelSDK from ${dynamixel_sdk_SOURCE_DIR}")
else()
  include(FetchContent)
  FetchContent_Declare(
    dynamixel_sdk
    GIT_REPOSITORY https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    GIT_TAG ${ROBOPY_DXL_SDK_TAG}
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR c++/include
  )
  FetchContent_MakeAvailable(dynamixel_sdk)
  message(STATUS "Fetched DynamixelSDK ${ROBOPY_DXL_SDK_TAG} into ${dynamixel_sdk_SOURCE_DIR}")
endif()

set(SDK_SRC "${dynamixel_sdk_SOURCE_DIR}/c++/src/dynamixel_sdk")
if(NOT EXISTS "${SDK_SRC}/protocol2_packet_handler.cpp")
  message(FATAL_ERROR "DynamixelSDK sources not found under ${SDK_SRC}")
endif()

set(SDK_SOURCES
  ${SDK_SRC}/group_bulk_read.cpp
  ${SDK_SRC}/group_bulk_write.cpp
  ${SDK_SRC}/group_fast_bulk_read.cpp
  ${SDK_SRC}/group_fast_sync_read.cpp
  ${SDK_SRC}/group_handler.cpp
  ${SDK_SRC}/group_sync_read.cpp
  ${SDK_SRC}/group_sync_write.cpp
  ${SDK_SRC}/packet_handler.cpp
  ${SDK_SRC}/port_handler.cpp
  ${SDK_SRC}/protocol1_packet_handler.cpp
  ${SDK_SRC}/protocol2_packet_handler.cpp
)
if(APPLE)
  list(APPEND SDK_SOURCES ${SDK_SRC}/port_handler_mac.cpp)
elseif(WIN32)
  list(APPEND SDK_SOURCES ${SDK_SRC}/port_handler_windows.cpp)
else()
  list(APPEND SDK_SOURCES ${SDK_SRC}/port_handler_linux.cpp)
endif()

add_library(dynamixel_sdk_static STATIC ${SDK_SOURCES})
target_include_directories(dynamixel_sdk_static
  PUBLIC "${dynamixel_sdk_SOURCE_DIR}/c++/include"
         "${dynamixel_sdk_SOURCE_DIR}/c++/include/dynamixel_sdk")
if(MSVC)
  target_compile_definitions(dynamixel_sdk_static PUBLIC WINDLLEXPORT)
else()
  target_compile_options(dynamixel_sdk_static PRIVATE -w)
endif()

# --- Extension ----------------------------------------------------------------
pybind11_add_module(_core MODULE
  src/module.cpp
  src/bus.cpp
  src/port_tuning.cpp
)
target_link_libraries(_core PRIVATE dynamixel_sdk_static)
target_compile_definitions(_core PRIVATE ROBOPY_DXL_VERSION="${SKBUILD_PROJECT_VERSION}")
if(NOT MSVC)
  target_compile_options(_core PRIVATE -O3 -Wall -Wextra)
endif()
find_package(Threads REQUIRED)
target_link_libraries(_core PRIVATE Threads::Threads)

install(TARGETS _core DESTINATION robopy_dxl)
