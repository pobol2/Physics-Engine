# The reusable engine library. Builds on its own or through add_subdirectory()
# from another project, and never pulls in examples, apps, or tests.
cmake_minimum_required(VERSION 3.20)
project(physics_engine_library VERSION 1.0.0
    DESCRIPTION "Classical particle mechanics: models, constraints, integrators, diagnostics"
    LANGUAGES CXX)
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

option(PHYSICS_FETCH_EIGEN "Fetch pinned Eigen if no exact installed package is found" ON)

# A parent project may already provide Eigen3::Eigen; reuse it if so.
if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 3.4.0 EXACT QUIET CONFIG)
endif()
if(NOT TARGET Eigen3::Eigen)
    if(NOT PHYSICS_FETCH_EIGEN)
        message(FATAL_ERROR "Eigen 3.4.0 not found. Set Eigen3_DIR or CMAKE_PREFIX_PATH to its installation, or enable PHYSICS_FETCH_EIGEN.")
    endif()
    include(FetchContent)
    FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
        # Eigen is header-only. Skip its subproject's global testing options.
        SOURCE_SUBDIR physics-header-only
    )
    FetchContent_MakeAvailable(eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}"
    )
endif()

add_library(physics_engine STATIC
    src/constraints.cpp src/forces.cpp src/model.cpp src/motion.cpp src/simulation.cpp)
add_library(physics::engine ALIAS physics_engine)
target_include_directories(physics_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(physics_engine PUBLIC cxx_std_17)
target_link_libraries(physics_engine PUBLIC Eigen3::Eigen)
# Settings stay on this target so a consuming project's flags are untouched.
set_target_properties(physics_engine PROPERTIES CXX_EXTENSIONS OFF)
if(MSVC)
    target_compile_options(physics_engine PRIVATE /W4)
else()
    target_compile_options(physics_engine PRIVATE -Wall -Wextra -Wpedantic)
endif()
