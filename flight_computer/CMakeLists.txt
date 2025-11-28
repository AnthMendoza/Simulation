cmake_minimum_required(VERSION 3.15)
project(flight_computer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(Threads REQUIRED)

set(SIM_SRC
    ../../simulation/src/sim/toml.cpp
    ../../simulation/src/utility/time_manager.cpp
)

set(TELEMETRY_SRC
    telemetry/src/communication.cpp
)

add_executable(flight_computer
    src/flight_computer.cpp
    ${TELEMETRY_SRC}
    ${SIM_SRC}
)

add_library(flight_computer_library STATIC
    ${TELEMETRY_SRC}
    ${SIM_SRC}
)

target_link_libraries(flight_computer 
    PRIVATE 
    flight_computer_library
    Threads::Threads
)