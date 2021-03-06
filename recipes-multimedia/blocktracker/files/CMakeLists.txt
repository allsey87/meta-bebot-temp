cmake_minimum_required(VERSION 3.1)
# Set project name
project(blocktracker)

# Set the module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Find dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(ISSCapture REQUIRED)
include_directories(${ISSCapture_INCLUDE_DIR})
find_package(AprilTag REQUIRED)
include_directories(${AprilTag_INCLUDE_DIR})

# Temporary include_directories for ARGoS 3 headers
include_directories("./src/argos3")

# Build the application
add_executable(blocktracker
               src/block.cc
               src/block_demo.cc
               src/block_sensor.cc
               src/block_tracker.cc
               src/frame_annotator.cc
               src/hungarian_solver.cc
               src/packet_control_interface.cc
               src/state.cc
               src/structure_analyser.cc
               src/tag.cc
               src/target.cc
               src/tcp_image_socket.cc
               src/uart_socket.cc
               src/argos3/core/utility/math/vector3.cpp
               src/argos3/core/utility/math/angles.cpp
               src/argos3/core/utility/math/matrix/rotationmatrix3.cpp)

set_property(TARGET blocktracker PROPERTY CXX_STANDARD 11)

target_link_libraries(blocktracker ${ISSCapture_LIB} ${AprilTag_LIB} ${OpenCV_LIBS})
install(TARGETS blocktracker DESTINATION bin)


