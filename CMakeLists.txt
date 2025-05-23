cmake_minimum_required(VERSION 3.15)
project(RocketSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Debugging and sanitizer flags
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)



# macOS deployment target for Apple Silicon (ASan requires macOS 11+ on arm64)
if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "arm64")
endif()


include_directories(
  include
  third_party/googletest/googletest/include
)

# Main executable
add_executable(rocket
    src/main.cpp
    src/vehicle.cpp
    src/vectorMath.cpp
    src/rotationMatrix.cpp
    src/odeIterator.cpp
    src/logs.cpp
    src/getRotationAngles.cpp
    src/forceApplied.cpp
    src/control.cpp
    src/aero.cpp
    src/RungeKutta.cpp
    src/sensors.cpp
    src/rocket.cpp
    src/toml.cpp
)

# Test executable
add_executable(tests
    src/vehicle.cpp
    src/vectorMath.cpp
    src/rotationMatrix.cpp
    src/odeIterator.cpp
    src/logs.cpp
    src/getRotationAngles.cpp
    src/forceApplied.cpp
    src/control.cpp
    src/aero.cpp
    src/RungeKutta.cpp
    src/sensors.cpp
    src/rocket.cpp
    src/toml.cpp
    tests/tests.cpp
)

# Static library
add_library(MySim STATIC
    src/MySim.cpp
    src/vehicle.cpp
    src/vectorMath.cpp
    src/rotationMatrix.cpp
    src/odeIterator.cpp
    src/logs.cpp
    src/getRotationAngles.cpp
    src/forceApplied.cpp
    src/control.cpp
    src/aero.cpp
    src/RungeKutta.cpp
    src/sensors.cpp
    src/rocket.cpp
    src/toml.cpp
)

# GoogleTest
add_subdirectory(third_party/googletest)

target_link_libraries(tests gtest gtest_main)

add_test(NAME tests COMMAND tests)
