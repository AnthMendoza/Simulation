find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
include(FetchContent)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
)

FetchContent_MakeAvailable(eigen)


FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.13.0  
)

FetchContent_MakeAvailable(spdlog)



if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()



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

add_library(core STATIC
    ${UTIL_SRC}
    ${BASE_SRC}
)

target_compile_definitions(core
    PUBLIC
    SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
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