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