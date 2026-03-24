cmake_minimum_required(VERSION 3.15)
project(flight_computer CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)
find_package(Threads REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(TELEMETRY_SRC
    telemetry/src/communication.cpp
    telemetry/src/communication_base.cpp
)

set(PID_CONTROLLER_SRC
    pid_controller/src/pid_controller.cpp
)

set(STATE_ESTIMATION_SRC
    state_estimation/src/state_estimation_bypass.cpp
)

add_library(flight_computer_library STATIC
    src/flight_computer.cpp
    ${TELEMETRY_SRC}
    ${PID_CONTROLLER_SRC}
    ${STATE_ESTIMATION_SRC}
)

target_include_directories(flight_computer_library
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/telemetry/include
    ${CMAKE_CURRENT_SOURCE_DIR}/state_estimation/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/pid_controller/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor/include

)

target_link_libraries(flight_computer_library
    PUBLIC
)

add_executable(drone 
    src/flight_computer_main.cpp
)

target_link_libraries(drone
    PUBLIC
    flight_computer_library
    Threads::Threads
)
