cmake_minimum_required(VERSION 3.12)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(kot_motor_lib VERSION 1.0.0 LANGUAGES CXX C)

set(SOURCES
  src/motor/motor.cpp

  src/controllers/basic.cpp
  src/controllers/direct_position.cpp
  src/controllers/direct_velocity.cpp
  src/controllers/direct_torque.cpp
  src/controllers/position_step.cpp
  src/controllers/velocity_accel.cpp

  src/transport/basic_transport.cpp
  src/transport/socketcan_transport.cpp

  src/dimensions/dimensions.cpp
)

add_library(
  kotmotor 
  STATIC 
  ${SOURCES}
)

target_compile_options(
  kotmotor
  PRIVATE

  -Wall
)

target_include_directories(
  kotmotor
  PUBLIC

  ${CMAKE_CURRENT_SOURCE_DIR} #kot_motor
  # maybe it's worth making all the paths from src?
  src
  src/controllers
  src/dimensions
  src/motor
  src/transport
)

target_include_directories(
  kotmotor
  INTERFACE

  include
)

set_target_properties(
  kotmotor 
  PROPERTIES

  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

# Optionally add an option to build examples
option(BUILD_EXAMPLES "Build examples" YES)

message(STATUS "build library with examples? ${BUILD_EXAMPLES}")

if(BUILD_EXAMPLES)
  add_subdirectory(examples/velocity_accel_controller)
endif()




