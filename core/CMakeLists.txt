find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Eigen3 3.4 QUIET CONFIG)
include(FetchContent)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)




if(NOT TARGET Eigen3::Eigen)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
    )
    FetchContent_Populate(eigen)
endif()

if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()



FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.13.0  
)

FetchContent_MakeAvailable(spdlog)


FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.9.0
)

FetchContent_MakeAvailable(yaml-cpp)





set(UTIL_SRC
    src/util/graphing.cpp
    src/util/valuelogging.cpp
    src/util/time_manager.cpp
    src/util/toml.cpp
    src/util/PIDController.cpp
)
set(BASE_SRC
    src/base/rotationMatrix.cpp
    src/base/odeIterator.cpp
    src/base/getRotationAngles.cpp
    src/base/forceApplied.cpp
    src/base/RungeKutta.cpp
    src/base/quaternion.cpp
    src/base/transpose.cpp
)

add_library(core
    STATIC
    ${UTIL_SRC}
    ${BASE_SRC}
)

target_compile_definitions(core
    PUBLIC
    SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
)


target_link_libraries(core 
    PUBLIC
        Eigen3::Eigen
        spdlog::spdlog_header_only
        ${Python3_LIBRARIES}
)

target_include_directories(core 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${Python3_INCLUDE_DIRS}
)

enable_testing()
add_executable(core_tests
    tests/test_main.cpp
    tests/time_manager_test.cpp
    tests/matrix_test.cpp
    tests/pose_rotation_test.cpp
    tests/quaternion_test.cpp
    tests/yaml_h_test.cpp
)

target_link_libraries(core_tests
    PUBLIC
    core
    gtest
    gtest_main
    yaml-cpp::yaml-cpp
)

add_test(NAME core_tests COMMAND core_tests)
