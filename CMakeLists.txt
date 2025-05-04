cmake_minimum_required(VERSION 3.15)
project(RocketSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)




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
    src/constants.cpp
    src/aero.cpp
    src/RungeKutta.cpp
    src/sensors.cpp
)

add_executable(unreal
    src/unreal.cpp
    src/vehicle.cpp
    src/vectorMath.cpp
    src/rotationMatrix.cpp
    src/odeIterator.cpp
    src/logs.cpp
    src/getRotationAngles.cpp
    src/forceApplied.cpp
    src/control.cpp
    src/constants.cpp
    src/aero.cpp
    src/RungeKutta.cpp
    src/sensors.cpp
)

add_executable(tests
    src/vehicle.cpp
    src/vectorMath.cpp
    src/rotationMatrix.cpp
    src/odeIterator.cpp
    src/logs.cpp
    src/getRotationAngles.cpp
    src/forceApplied.cpp
    src/control.cpp
    src/constants.cpp
    src/aero.cpp
    src/RungeKutta.cpp
    src/sensors.cpp
    tests/tests.cpp
)
add_subdirectory(third_party/googletest)

include_directories(
  include
  third_party/googletest/googletest/include
  
)

target_link_libraries(tests gtest gtest_main)


add_test(NAME tests COMMAND tests)
