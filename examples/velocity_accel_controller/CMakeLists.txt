cmake_minimum_required(VERSION 3.12)

project(VelocityAccelControllerExample LANGUAGES CXX C)

# could be used to plase an executable in the same dir as the src of the example.
# is it a good practice? 
set(EXAMPLE_BINARY_DIR ${CMAKE_SOURCE_DIR}/examples/velocity_accel_controller)
set(CMAKE_BINARY_DIR ${EXAMPLE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${EXAMPLE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${EXAMPLE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXAMPLE_BINARY_DIR})

add_executable(
  exec 
  main.cpp
)

target_compile_options(
  exec
  PRIVATE

  -Wall
)

target_link_libraries(
  exec 
  PRIVATE 
  kotmotor
)

target_include_directories(
  exec 
  PRIVATE 
  ${CMAKE_SOURCE_DIR}/include
)




