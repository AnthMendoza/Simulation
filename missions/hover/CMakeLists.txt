
add_executable(hover
    main.cpp
)


target_link_libraries(hover
    PUBLIC
        flight_computer_library
        MySim
)

target_compile_definitions(hover PUBLIC 
    PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
    HOVER_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_include_directories(hover
    PRIVATE
        ${CMAKE_SOURCE_DIR}/simulation/include/sim
)


