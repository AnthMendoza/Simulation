find_package(Threads REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.9.0
)

FetchContent_MakeAvailable(yaml-cpp)


FetchContent_Declare(
  GeographicLib
  GIT_REPOSITORY https://github.com/geographiclib/geographiclib.git
  GIT_TAG        v2.3
)

FetchContent_MakeAvailable(GeographicLib)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
)

set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)




set(TELEMETRY_SRC
    telemetry/src/communication.cpp
    telemetry/src/communication_base.cpp
)

set(PID_CONTROLLER_SRC
    pid_controller/src/pid_controller.cpp
    pid_controller/src/pid_config_parse.cpp
)

set(STATE_ESTIMATION_SRC
    state_estimation/src/state_estimation_base.cpp
    state_estimation/src/estimation.cpp
)

set(HARDWARE_SRC
    hardware/src/hardware_base.cpp
    hardware/src/gps.cpp
    hardware/src/airframe_config.cpp

)

add_library(flight_computer_library STATIC
    src/flight_computer.cpp
    src/drone_control.cpp
    ${TELEMETRY_SRC}
    ${PID_CONTROLLER_SRC}
    ${STATE_ESTIMATION_SRC}
    ${HARDWARE_SRC}
)

target_include_directories(flight_computer_library
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/telemetry/include
    ${CMAKE_CURRENT_SOURCE_DIR}/state_estimation/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/pid_controller/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/hardware/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)



target_link_libraries(flight_computer_library
    PUBLIC
    core
    GeographicLib::GeographicLib
    yaml-cpp::yaml-cpp
    Threads::Threads
)

add_executable(drone 
    src/flight_computer_main.cpp
)

target_link_libraries(drone
    PUBLIC
    flight_computer_library
)

enable_testing()
add_executable(flight_tests
    tests/flight_computer_test.cpp
    tests/airframe_config_parser_test.cpp
)

target_link_libraries(flight_tests
    PUBLIC
    flight_computer_library
)

target_link_libraries(flight_tests
    PRIVATE
    gtest
    gtest_main
)
add_test(NAME flight_tests COMMAND flight_tests)
