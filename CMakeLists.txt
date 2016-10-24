#
# Laputa VR plugin for OSVR
#
cmake_minimum_required(VERSION 3.0.0)
project(com_osvr_Laputa)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(osvr REQUIRED)
find_package(VRPN REQUIRED)

# Generate our JSON header file
osvr_convert_json(com_osvr_Laputa_json
    com_osvr_Laputa.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_Laputa_json.h")

osvr_convert_json(com_osvr_IMU_Combiner_json
    com_osvr_IMU_Combiner.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_IMU_Combiner_json.h")

add_subdirectory(vendor)

if(NOT TARGET hidapi)
    message(FATAL_ERROR "HIDAPI is required to build this project!")
endif()

# Be able to find our generated header file.
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# Build and install the plugin
osvr_add_plugin(com_osvr_Laputa
    com_osvr_Laputa.cpp
    VRPNMultiserver.h
    VRPNMultiserver.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_Laputa_json.h"
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_IMU_Combiner_json.h")

target_include_directories(com_osvr_Laputa
                            PRIVATE
                            ${VRPN_INCLUDE_DIRS}
                            ${CMAKE_CURRENT_BINARY_DIR})

# Link against PluginKit (C++ wrapper), Util, and hidapi
target_link_libraries(com_osvr_Laputa
    osvr::osvrPluginKitCpp
    osvr::osvrUtilCpp
    hidapi
     ${VRPN_SERVER_LIBRARIES}
    osvr::osvrVRPNServer)

# Install docs, license, sample config
# install(TARGETS com_osvr_Laputa
    # LIBRARY DESTINATION .)
# INSTALL(FILES
    # osvr_server_config.laputa.sample.json
    # README.md
    # LICENSE
    # DESTINATION .)