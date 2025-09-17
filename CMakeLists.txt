cmake_minimum_required(VERSION 3.15)
project(grmblinkpy LANGUAGES CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pybind11_add_module(_core
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/mblink.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/mblink_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/mblink_socket.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/app_interface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/data_dicts.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/ping_pong.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/modemgr/modemgr.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/modemgr/modemgr_client.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/mblinkpy_bindings.cpp
)

if(APPLE)
  target_compile_definitions(_core PRIVATE
    MBLINK_NO_RT_SCHED=1
    MBLINK_NO_CPU_AFFINITY=1
  )
endif()

# EIGEN (vendored)
set(EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cpp/thirdparty/eigen")
if(NOT EXISTS "${EIGEN_DIR}/Eigen/Core")
  message(FATAL_ERROR "Eigen not found at ${EIGEN_DIR}. Put headers there or enable USE_SYSTEM_EIGEN.")
endif()
target_include_directories(_core PRIVATE "${EIGEN_DIR}")

# MAVLink (vendored)
set(MAVLINK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cpp/thirdparty/mavlink")
if(NOT EXISTS "${MAVLINK_DIR}/common/mavlink.h" AND NOT EXISTS "${MAVLINK_DIR}/mavlink/common/mavlink.h")
  message(FATAL_ERROR "MAVLink headers not found under ${MAVLINK_DIR}. Expected common/mavlink.h or mavlink/common/mavlink.h")
endif()
# The include root must be the parent of 'mavlink/common/mavlink.h'
# Handle both layouts:
if(EXISTS "${MAVLINK_DIR}/mavlink/common/mavlink.h")
  target_include_directories(_core PRIVATE "${MAVLINK_DIR}")
else()
  # c_library_v2 has 'common/mavlink.h' at root; include that root as 'mavlink'
  target_include_directories(_core PRIVATE "${MAVLINK_DIR}/..")
endif()


target_include_directories(_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/modemgr
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/thirdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/thirdparty/eigen
)

if (MSVC)
  # add MSVC flags if you want
else()
  target_compile_options(_core PRIVATE -Wno-address-of-packed-member -fvisibility=hidden)
endif()
target_compile_definitions(_core PRIVATE EIGEN_NO_DEBUG)

if (WIN32)
  target_link_libraries(_core PRIVATE ws2_32)
else()
  target_link_libraries(_core PRIVATE pthread)
endif()

# put the .so/.pyd inside the Python package on install
install(TARGETS _core LIBRARY DESTINATION grmblinkpy)
