cmake_minimum_required(VERSION 3.15)
project(flight_computer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(Threads REQUIRED)

add_executable(flight_computer
    src/flight_computer.cpp
    telemetry/src/communication.cpp
    ../../simulation/src/sim/toml.cpp
)

target_link_libraries(flight_computer 
    PRIVATE 
    Threads::Threads
)