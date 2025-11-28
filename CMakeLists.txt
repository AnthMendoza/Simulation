cmake_minimum_required(VERSION 3.15)
project(MainProject CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "arm64")
endif()

add_subdirectory(flight_computer)
add_subdirectory(simulation)

