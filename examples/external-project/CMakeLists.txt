# A stand-alone project that uses the engine the way an external repository
# (for example, a set of Landau and Lifshitz problems) would: it adds engine/
# with add_subdirectory and links one target. Nothing else from Physics-Engine
# is configured or built.
cmake_minimum_required(VERSION 3.20)
project(pendulum_study LANGUAGES CXX)

# Point this at the engine/ directory of a Physics-Engine checkout, for example
# a git submodule at external/Physics-Engine/engine.
set(PHYSICS_ENGINE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../engine"
    CACHE PATH "Path to Physics-Engine/engine")
add_subdirectory("${PHYSICS_ENGINE_DIR}" physics-engine EXCLUDE_FROM_ALL)

add_executable(pendulum_study main.cpp)
target_link_libraries(pendulum_study PRIVATE physics::engine)
