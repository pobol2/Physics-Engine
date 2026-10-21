cmake_minimum_required(VERSION 3.20)
project(physics_engine VERSION 1.0.0
    DESCRIPTION "General-purpose C++ engine for classical particle mechanics"
    HOMEPAGE_URL "https://github.com/pobol2/Physics-Engine"
    LANGUAGES CXX)
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# Orchestrates the repository. The reusable library lives in engine/ and builds
# without anything else here; examples, apps, and tests are layered on top.
option(PHYSICS_BUILD_VIEWER "Build the optional OpenGL viewer" OFF)
option(PHYSICS_FETCH_GLFW "Fetch pinned GLFW if no installed package is found" ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Front ends and test programs land at the top of the build tree.
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
endif()

function(physics_project_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_subdirectory(engine)
add_subdirectory(examples)
add_subdirectory(apps)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
