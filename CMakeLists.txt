cmake_minimum_required(VERSION 3.16)

project(minimal_hik_gimbal_bridge LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

set(SERIAL_LIB_SOURCES
  third_party/serial/src/serial.cc
  third_party/serial/src/impl/unix.cc
)

set(HIKROBOT_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikrobot" CACHE PATH "Path to the vendored HikRobot SDK")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(HIKROBOT_LIB_ARCH arm64)
else()
  set(HIKROBOT_LIB_ARCH amd64)
endif()

set(HIKROBOT_INCLUDE_DIR "${HIKROBOT_SDK_DIR}/include")
set(HIKROBOT_LIBRARY "${HIKROBOT_SDK_DIR}/lib/${HIKROBOT_LIB_ARCH}/libMvCameraControl.so")

if(NOT EXISTS "${HIKROBOT_INCLUDE_DIR}/MvCameraControl.h")
  message(FATAL_ERROR "HikRobot SDK headers not found under ${HIKROBOT_INCLUDE_DIR}; expected vendored copy in third_party/hikrobot or override HIKROBOT_SDK_DIR")
endif()

if(NOT EXISTS "${HIKROBOT_LIBRARY}")
  message(FATAL_ERROR "HikRobot SDK library not found at ${HIKROBOT_LIBRARY}; expected vendored copy in third_party/hikrobot or override HIKROBOT_SDK_DIR")
endif()

add_library(minimal_hik_gimbal_bridge_core STATIC
  bridge/camera_preview.cpp
  bridge/common.cpp
  bridge/frame.cpp
  bridge/frame_preprocessor.cpp
  bridge/h264_encoder.cpp
  bridge/hik_camera.cpp
  bridge/options.cpp
  bridge/serial_port.cpp
  bridge/udp_sender.cpp
  ${SERIAL_LIB_SOURCES}
)

target_include_directories(minimal_hik_gimbal_bridge_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(minimal_hik_gimbal_bridge_core PRIVATE
  ${HIKROBOT_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/serial/include
)

target_link_libraries(minimal_hik_gimbal_bridge_core PUBLIC
  ${HIKROBOT_LIBRARY}
  ${OpenCV_LIBS}
  Threads::Threads
  rt
  dl
)

add_executable(minimal_hik_gimbal_bridge
  src/main.cpp
)

target_link_libraries(minimal_hik_gimbal_bridge PRIVATE
  minimal_hik_gimbal_bridge_core
)

set_target_properties(minimal_hik_gimbal_bridge PROPERTIES
  BUILD_RPATH "${HIKROBOT_SDK_DIR}/lib/${HIKROBOT_LIB_ARCH}"
)