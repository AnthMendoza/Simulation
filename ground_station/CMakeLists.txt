
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(Threads REQUIRED)

add_executable(ground_station
    src/ground_station.cpp
    telemetry/src/telemetry_ground.cpp
    gui/src/gui_connector.cpp
    src/ground_station_startup.cpp
)

target_link_libraries(ground_station
    PUBLIC
    core
    flight_computer_library
    Threads::Threads
)