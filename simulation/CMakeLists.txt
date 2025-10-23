cmake_minimum_required(VERSION 3.15)
project(RocketSim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)

# macOS-specific
if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "arm64")
endif()

# Enable compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Python (for Python.h)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Fetch Third-Party Libraries
include(FetchContent)

# ---- GoogleTest ----
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ---- Eigen3 ----
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

# Global Include Directories
include_directories(
    include
    ${Python3_INCLUDE_DIRS}
)

# Source Groups
set(SIM_SRC
    src/sim/logs.cpp
    src/sim/toml.cpp
    #src/sim/MySim.cpp
)

set(CORE_SRC
    src/core/rotationMatrix.cpp
    src/core/odeIterator.cpp
    src/core/getRotationAngles.cpp
    src/core/forceApplied.cpp
    src/core/RungeKutta.cpp
    src/core/quaternion.cpp
)

set(DYNAMICS_SRC
    src/dynamics/vehicle.cpp
    src/dynamics/aero.cpp
    #src/dynamics/rocket.cpp
    src/dynamics/drone.cpp
)

set(CONTROL_SRC
    src/control/control.cpp
    src/control/PIDController.cpp
    src/control/droneControl.cpp
    src/control/dronePIDControl.cpp
    src/control/stateEstimation.cpp
    src/control/testGlobals.cpp
)

set(SUBSYSTEMS_SRC
    src/subsystems/sensors.cpp
    src/subsystems/battery.cpp
    src/subsystems/motor.cpp
    src/subsystems/droneDependencyInjector.cpp
    src/subsystems/droneSensorSuite.cpp
)

set(UTILITY_SRC
    src/utility/time_manager.cpp
    src/utility/logger.cpp
)

# Static Library
add_library(MySim STATIC
    ${SIM_SRC}
    ${CORE_SRC}
    ${DYNAMICS_SRC}
    ${CONTROL_SRC}
    ${SUBSYSTEMS_SRC}
    ${UTILITY_SRC}
)

target_include_directories(MySim
    PUBLIC
    include
    ${Python3_INCLUDE_DIRS}
)

target_link_libraries(MySim
    PUBLIC
    Eigen3::Eigen
    ${Python3_LIBRARIES}
)

# Executables
#add_executable(rocket
#    src/sim/rocketMain.cpp
#)

add_executable(drone
    src/sim/droneSim.cpp
)

#add_executable(dronePIDGains
#    src/sim/dronePIDGains.cpp
#)


# Tests
enable_testing()
add_executable(tests
    tests/tests.cpp
    tests/testPoseRotation.cpp
    tests/quaternionTests.cpp
    tests/timeManagerTest.cpp
    tests/stateEstimationTest.cpp
    tests/PIDControllerTest.cpp
    tests/momentValidationTest.cpp
)

# Link MySim and Python into all executables
foreach(TARGET  drone tests)
    target_link_libraries(${TARGET}
        PRIVATE
        MySim
        pthread
    )
endforeach()

# GoogleTest for tests
target_link_libraries(tests PRIVATE gtest gtest_main)
add_test(NAME RocketSimTests COMMAND tests)
