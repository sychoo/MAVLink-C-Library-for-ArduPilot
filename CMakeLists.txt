cmake_minimum_required(VERSION 3.13)

#project(udp_example C)
project (demo C)

set(CMAKE_C_STANDARD 11)

find_package(MAVLink REQUIRED)

#add_executable(udp_example udp_example.c)
add_executable(demo demo.c)

#target_link_libraries(udp_example PRIVATE MAVLink::mavlink)
target_link_libraries(demo PRIVATE MAVLink::mavlink)
