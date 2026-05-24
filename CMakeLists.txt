cmake_minimum_required(VERSION 3.16)
project(qmini_sdk VERSION 1.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(BACKEND "sim" CACHE STRING "HAL backend: sim | hardware")
set_property(CACHE BACKEND PROPERTY STRINGS sim hardware)
message(STATUS "Building with BACKEND=${BACKEND}")

option(WITH_ONNX  "Link onnxruntime (real inference)"           ON)
option(WITH_TESTS "Build unit + integration tests"              ON)

# ---------------------------------------------------------------------------
# Above-HAL sources: pure C++, no DDS / no Python.
# ---------------------------------------------------------------------------
set(USER_SOURCES
    source/user/obs_builder.cpp
    source/user/rl_controller.cpp
    source/user/data_report.cpp
    source/user/qmini_app.cpp
    source/user/policy_identity.cpp
)

if (WITH_ONNX)
    list(APPEND USER_SOURCES source/user/policy_onnx.cpp)
else()
    list(APPEND USER_SOURCES source/user/policy_onnx_stub.cpp)
endif()

# ---------------------------------------------------------------------------
# HAL backends — exactly one set of *.cpp added below.
# ---------------------------------------------------------------------------
if (BACKEND STREQUAL "sim")
    list(APPEND USER_SOURCES
        source/user/hal/sim/clock_std.cpp
        source/user/hal/sim/motor_sim.cpp
        source/user/hal/sim/imu_sim.cpp
        source/user/hal/sim/joystick_keyboard.cpp
    )
elseif (BACKEND STREQUAL "hardware")
    list(APPEND USER_SOURCES
        source/user/hal/hardware/clock_unitree.cpp
        source/user/hal/hardware/motor_unitree.cpp
        source/user/hal/hardware/imu_serial.cpp
        source/user/hal/hardware/joystick_linux.cpp
    )
else()
    message(FATAL_ERROR "Unknown BACKEND=${BACKEND}; use sim or hardware")
endif()

add_library(qmini_user STATIC ${USER_SOURCES})

target_include_directories(qmini_user PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/user
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
)

include(FetchContent)

# Eigen: header-only, prefer system install, fall back to FetchContent.
find_package(Eigen3 QUIET)
if (NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found system-wide; fetching 3.4.0")
    FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72)
    # We don't want Eigen's tests / installs polluting the build.
    set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen)
    add_library(eigen_iface INTERFACE)
    target_include_directories(eigen_iface SYSTEM INTERFACE
        ${eigen_SOURCE_DIR})
    add_library(Eigen3::Eigen ALIAS eigen_iface)
endif()
target_link_libraries(qmini_user PUBLIC Eigen3::Eigen)

find_package(Threads REQUIRED)
target_link_libraries(qmini_user PUBLIC Threads::Threads)

# yaml-cpp: prefer system, fall back to FetchContent.
find_package(yaml-cpp QUIET)
if (NOT yaml-cpp_FOUND AND NOT TARGET yaml-cpp)
    message(STATUS "yaml-cpp not found system-wide; fetching 0.8.0")
    FetchContent_Declare(yaml_cpp
        URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz)
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(yaml_cpp)
endif()
target_link_libraries(qmini_user PUBLIC yaml-cpp)

if (WITH_ONNX)
    target_compile_definitions(qmini_user PUBLIC QMINI_WITH_ONNX=1)
    target_include_directories(qmini_user PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/onnx)
    # Find onnxruntime: try vendored lib first (arm64 only), then system.
    find_library(ONNXRUNTIME_LIB onnxruntime
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/onnx
              /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
        NO_DEFAULT_PATH)
    if (NOT ONNXRUNTIME_LIB)
        find_library(ONNXRUNTIME_LIB onnxruntime)
    endif()
    if (NOT ONNXRUNTIME_LIB)
        message(WARNING
            "onnxruntime not found; identity policy still available.")
    else()
        message(STATUS "onnxruntime: ${ONNXRUNTIME_LIB}")
        target_link_libraries(qmini_user PUBLIC ${ONNXRUNTIME_LIB})
    endif()
endif()

# ---------------------------------------------------------------------------
# Hardware-only deps (Unitree SDK, motor SDK)
# ---------------------------------------------------------------------------
if (BACKEND STREQUAL "hardware")
    find_package(jsoncpp QUIET)
    if (jsoncpp_FOUND)
        target_link_libraries(qmini_user PUBLIC JsonCpp::JsonCpp)
    endif()
    set(UNITREE_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
    find_library(UNITREE_SDK_LIB unitree_sdk2
        PATHS ${UNITREE_SDK_PATH} NO_DEFAULT_PATH)
    if (NOT UNITREE_SDK_LIB)
        message(FATAL_ERROR
            "unitree_sdk2 not found at ${UNITREE_SDK_PATH}")
    endif()
    add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
    set_target_properties(unitree_sdk2 PROPERTIES
        IMPORTED_LOCATION ${UNITREE_SDK_LIB})
    target_link_libraries(qmini_user PUBLIC unitree_sdk2 ddsc ddscxx rt)
    # Motor SDK
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        target_link_libraries(qmini_user PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/m8010motor/libUnitreeMotorSDK_Arm64.so)
    else()
        target_link_libraries(qmini_user PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/m8010motor/libUnitreeMotorSDK_Linux64.so)
    endif()
endif()

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(run_interface source/run_interface.cpp)
target_link_libraries(run_interface PRIVATE qmini_user)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if (WITH_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
