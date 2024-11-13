cmake_minimum_required(VERSION 3.10)

# Define the project name
project(RocketLanding)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


set(SOURCES
    src/main.cpp
    src/vehicle.cpp
    src/vectorMath.cpp
    src/odeIterator.cpp
    src/aero.cpp
    src/RungeKutta.cpp
    src/logs.cpp
    src/forceApplied.cpp
    src/rotationMatrix.cpp
    src/constants.cpp
    src/getRotationAngles.cpp
    src/control.cpp
)


add_executable(MyApp ${SOURCES})


set_target_properties(MyApp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build
)
