cmake_minimum_required(VERSION 3.10.0)

project(phys_engine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
    SDL2
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-2.30.11
)
FetchContent_MakeAvailable(SDL2)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.0
)
FetchContent_MakeAvailable(Catch2)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC SDL2::SDL2)

add_library(phys INTERFACE)
target_include_directories(phys INTERFACE include/)

add_executable(Game
    src/main.cpp
    src/Scene.cpp
    src/Renderer.cpp
    include/phys/Object.hpp
    include/phys/Circle.hpp
    include/phys/Manifold.hpp
    include/phys/AABB.hpp
)
target_link_libraries(Game PRIVATE phys SDL2::SDL2 SDL2::SDL2main imgui_lib)

if(WIN32)
    target_link_options(Game PRIVATE "/SUBSYSTEM:CONSOLE")
    add_custom_command(TARGET Game POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:Game>
    )
endif()

# tests
enable_testing()

add_executable(tests
    tests/test_vec2.cpp
    tests/test_object.cpp
    tests/test_circle.cpp
    tests/test_scene.cpp
    src/Scene.cpp
)
target_include_directories(tests PRIVATE include/)
target_link_libraries(tests PRIVATE phys Catch2::Catch2WithMain)

add_test(NAME Vec2test COMMAND tests)