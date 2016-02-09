cmake_minimum_required(VERSION 2.8)
# Set project name
project(apriltag)

# Find dependencies
find_package(Threads)

# Create and install the Apriltag library
file(GLOB APRILTAG_SOURCE_FILES "src/*.c")
file(GLOB APRILTAG_HEADER_FILES "src/*.h")

# Set standard to C99 mode and enable reentrant options in standard libraries
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -pthread -O4")

# Add and install shared library
add_library(apriltag SHARED ${APRILTAG_SOURCE_FILES})
target_link_libraries(apriltag ${CMAKE_THREAD_LIBS_INIT} m)
set_target_properties(apriltag PROPERTIES SOVERSION 0.0.0
  VERSION 0.0.1)
install(TARGETS apriltag DESTINATION lib)

# Install header files
install (FILES ${APRILTAG_HEADER_FILES} DESTINATION include/apriltag)


