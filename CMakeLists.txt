cmake_minimum_required(VERSION 3.10.0)

project(phys_engine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.0
)
FetchContent_MakeAvailable(Catch2)

add_library(phys INTERFACE)
target_include_directories(phys INTERFACE include/)

add_executable(Game
    game/main.cpp
)
target_link_libraries(Game PRIVATE phys)

# tests
enable_testing()

add_executable(tests
    tests/test_vec2.cpp
)
target_link_libraries(tests PRIVATE phys Catch2::Catch2WithMain)

add_test(NAME Vec2test COMMAND tests)