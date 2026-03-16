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
    ../../simulation/src/control/PIDController.cpp
)

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

add_executable(flight_computer
    src/flight_computer.cpp
    src/flight_computer_main.cpp
    ${TELEMETRY_SRC}
    ${SIM_SRC}
    ${PID_CONTROLLER_SRC}
    ${STATE_ESTIMATION_SRC}
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