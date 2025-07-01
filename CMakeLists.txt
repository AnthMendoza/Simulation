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

# Rocket executable
add_executable(rocket
    src/sim/rocketMain.cpp
    src/dynamics/vehicle.cpp
    src/core/rotationMatrix.cpp
    src/core/odeIterator.cpp
    src/sim/logs.cpp
    src/core/getRotationAngles.cpp
    src/core/forceApplied.cpp
    src/control/control.cpp
    src/dynamics/aero.cpp
    src/core/RungeKutta.cpp
    src/subsystems/sensors.cpp
    src/dynamics/rocket.cpp
    src/sim/toml.cpp
    src/core/quaternion.cpp
)

# Drone Executable
add_executable(drone
    src/sim/droneMain.cpp
    src/dynamics/vehicle.cpp
    src/core/rotationMatrix.cpp
    src/core/odeIterator.cpp
    src/sim/logs.cpp
    src/core/getRotationAngles.cpp
    src/core/forceApplied.cpp
    src/control/control.cpp
    src/dynamics/aero.cpp
    src/core/RungeKutta.cpp
    src/subsystems/sensors.cpp
    src/dynamics/rocket.cpp
    src/sim/toml.cpp
    src/core/quaternion.cpp
    src/subsystems/battery.cpp
    src/subsystems/motor.cpp
    src/dynamics/drone.cpp
    src/control/PIDController.cpp
    src/control/droneControl.cpp
    src/sim/MySim.cpp
)

# Test executable
add_executable(tests
    src/dynamics/vehicle.cpp
    src/core/rotationMatrix.cpp
    src/core/odeIterator.cpp
    src/sim/logs.cpp
    src/core/getRotationAngles.cpp
    src/core/forceApplied.cpp
    src/control/control.cpp
    src/dynamics/aero.cpp
    src/core/RungeKutta.cpp
    src/subsystems/sensors.cpp
    src/dynamics/rocket.cpp
    src/sim/toml.cpp
    tests/tests.cpp
    src/core/quaternion.cpp
    src/control/PIDController.cpp
    src/control/droneControl.cpp
    src/dynamics/drone.cpp
    src/subsystems/motor.cpp
    src/subsystems/battery.cpp
)

# Static library
add_library(MySim STATIC
    src/sim/MySim.cpp
    src/dynamics/vehicle.cpp
    src/core/rotationMatrix.cpp
    src/core/odeIterator.cpp
    src/sim/logs.cpp
    src/core/getRotationAngles.cpp
    src/core/forceApplied.cpp
    src/control/control.cpp
    src/dynamics/aero.cpp
    src/core/RungeKutta.cpp
    src/subsystems/sensors.cpp
    src/dynamics/rocket.cpp
    src/sim/toml.cpp
    src/core/quaternion.cpp
    src/subsystems/battery.cpp
    src/subsystems/motor.cpp
    src/dynamics/drone.cpp
    src/control/PIDController.cpp
    src/control/droneControl.cpp
)


# GoogleTest
add_subdirectory(third_party/googletest)

target_link_libraries(tests gtest gtest_main)


add_test(NAME RocketSimTests COMMAND tests)
