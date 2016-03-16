cmake_minimum_required(VERSION 2.8)
# Set project name
project(isscapture)

# Set the module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Find dependencies
find_package(AprilTag REQUIRED)
include_directories(${AprilTag_INCLUDE_DIR})

# Create and install the ISS capture library
add_library(isscapture SHARED src/iss_capture.cc)
target_link_libraries(isscapture ${AprilTag_LIB})
set_target_properties(isscapture PROPERTIES SOVERSION 0.0.0
  VERSION 0.0.1)
install(TARGETS isscapture DESTINATION lib)
install(FILES src/iss_capture.h DESTINATION include)
