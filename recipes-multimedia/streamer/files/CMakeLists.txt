cmake_minimum_required(VERSION 3.1)
# Set project name
project(streamer)

# Set the module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Find dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(ISSCapture REQUIRED)
include_directories(${ISSCapture_INCLUDE_DIR})

# Build the application
add_executable(streamer 	src/streamer.cc
				src/tcp_image_socket.cc)

set_property(TARGET streamer PROPERTY CXX_STANDARD 11)

target_link_libraries(streamer ${ISSCapture_LIB} ${OpenCV_LIBS})
install(TARGETS streamer DESTINATION bin)


