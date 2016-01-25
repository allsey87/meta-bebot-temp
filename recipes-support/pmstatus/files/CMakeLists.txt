cmake_minimum_required(VERSION 3.1)
# Set project name
project(pmstatus)

# Build the application
add_executable(pmstatus
               src/pmstatus.cc
				   src/packet_control_interface.cc
				   src/uart_socket.cc)

set_property(TARGET pmstatus PROPERTY CXX_STANDARD 11)

install(TARGETS pmstatus DESTINATION bin)


