cmake_minimum_required(VERSION 3.16)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5) # work around for yaml-cpp
project(Flight_System CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)

if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "arm64")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})

add_subdirectory(core)
add_subdirectory(flight_computer)
add_subdirectory(simulation)
add_subdirectory(ground_station)
add_subdirectory(missions/hover)

# flight_computer's PID wrapper depends on the PID implementation in MySim.
# Link after both subdirectories are added so both targets exist.
# *note PID wrapper is only used in the flight computer tests, so this doesn't create a dependency for the flight computer library itself.
target_link_libraries(drone PUBLIC MySim)
