add_executable(hover
    main.cpp
)


target_link_libraries(hover
    PRIVATE
        flight_computer_library
        MySim
)

target_compile_definitions(hover PRIVATE 
    PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
    HOVER_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)


