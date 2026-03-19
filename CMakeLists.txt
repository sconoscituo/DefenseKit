cmake_minimum_required(VERSION 3.16)
project(DefenseKit VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

include_directories(src)

# Core library sources
set(DEFENSEKIT_SOURCES
    src/ballistics/projectile.cpp
    src/signal_processing/fft.cpp
    src/signal_processing/kalman_filter.cpp
    src/radar/target_tracker.cpp
    src/navigation/ins.cpp
)

add_library(defensekit_lib STATIC ${DEFENSEKIT_SOURCES})

# Examples
add_executable(ballistics_demo examples/ballistics_demo.cpp)
target_link_libraries(ballistics_demo defensekit_lib)

add_executable(radar_demo examples/radar_demo.cpp)
target_link_libraries(radar_demo defensekit_lib)

# Tests
enable_testing()
add_subdirectory(tests)
