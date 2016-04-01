cmake_minimum_required(VERSION 3.1)
# Set project name
project(rftest)

# Build the application
add_executable(rftest
               src/main.cc
               src/packet_control_interface.cc
               src/uart_socket.cc)

set_property(TARGET rftest PROPERTY CXX_STANDARD 11)

install(TARGETS rftest DESTINATION bin)


