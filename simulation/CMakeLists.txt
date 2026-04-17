
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



# Source Groups
set(SIM_SRC
    src/sim/logs.cpp
    src/sim/hardware_sim.cpp
    #src/sim/MySim.cpp
)



set(DYNAMICS_SRC
    src/dynamics/vehicle.cpp
    src/dynamics/droneMinimal.cpp
    src/dynamics/aero.cpp
    #src/dynamics/rocket.cpp
    #src/dynamics/drone.cpp
)

set(CONTROL_SRC
    #src/control/droneControl.cpp
    #src/control/dronePIDControl.cpp
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



# Static Library
add_library(MySim STATIC
    ${SIM_SRC}
    ${DYNAMICS_SRC}
    ${CONTROL_SRC}
    ${SUBSYSTEMS_SRC}
    ${UTILITY_SRC}
)

target_link_libraries(MySim
    PUBLIC
        core
        flight_computer_library
)


target_include_directories(MySim
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Tests
#enable_testing()
#add_executable(tests
#
#)

# Link MySim and Python into all executables
foreach(TARGET )
    target_link_libraries(${TARGET}
        PUBLIC
        MySim
        core
        pthread
    )
endforeach()

# GoogleTest for tests
#target_link_libraries(tests PUBLIC gtest gtest_main)
#add_test(NAME SimTests COMMAND tests)
