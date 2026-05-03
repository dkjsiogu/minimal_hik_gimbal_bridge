cmake_minimum_required(VERSION 3.16)

project(minimal_hik_gimbal_bridge LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(SERIAL_LIB_SOURCES
  third_party/serial/src/serial.cc
  third_party/serial/src/impl/unix.cc
)

set(HIKROBOT_SDK_DIR "/home/dkjsiogu/文档/sp_vision_25/io/hikrobot" CACHE PATH "Path to the bundled HikRobot SDK")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(HIKROBOT_LIB_ARCH arm64)
else()
  set(HIKROBOT_LIB_ARCH amd64)
endif()

set(HIKROBOT_INCLUDE_DIR "${HIKROBOT_SDK_DIR}/include")
set(HIKROBOT_LIBRARY "${HIKROBOT_SDK_DIR}/lib/${HIKROBOT_LIB_ARCH}/libMvCameraControl.so")

if(NOT EXISTS "${HIKROBOT_INCLUDE_DIR}/MvCameraControl.h")
  message(FATAL_ERROR "HikRobot SDK headers not found under ${HIKROBOT_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${HIKROBOT_LIBRARY}")
  message(FATAL_ERROR "HikRobot SDK library not found at ${HIKROBOT_LIBRARY}")
endif()

add_executable(minimal_hik_gimbal_bridge
  src/main.cpp
  ${SERIAL_LIB_SOURCES}
)

target_include_directories(minimal_hik_gimbal_bridge PRIVATE
  ${HIKROBOT_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/serial/include
)

target_link_libraries(minimal_hik_gimbal_bridge PRIVATE
  ${HIKROBOT_LIBRARY}
  ${OpenCV_LIBS}
  Threads::Threads
  rt
  dl
)

set_target_properties(minimal_hik_gimbal_bridge PROPERTIES
  BUILD_RPATH "${HIKROBOT_SDK_DIR}/lib/${HIKROBOT_LIB_ARCH}"
)