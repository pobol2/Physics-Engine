# Front-end support owns argument parsing and file output, outside the engine.
add_library(physics_app STATIC common.cpp)
target_include_directories(physics_app PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(physics_app PUBLIC physics::engine physics_examples)
physics_project_warnings(physics_app)

add_executable(physics_runner runner.cpp)
target_link_libraries(physics_runner PRIVATE physics_app)
physics_project_warnings(physics_runner)

if(PHYSICS_BUILD_VIEWER)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 3.3 QUIET CONFIG)
    if(TARGET glfw)
        set(PHYSICS_GLFW_TARGET glfw)
    elseif(TARGET glfw3::glfw)
        set(PHYSICS_GLFW_TARGET glfw3::glfw)
    elseif(PHYSICS_FETCH_GLFW)
        include(FetchContent)
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "Build GLFW documentation")
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "Build GLFW tests")
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build GLFW examples")
        set(GLFW_INSTALL OFF CACHE BOOL "Install GLFW")
        FetchContent_Declare(glfw
            GIT_REPOSITORY https://github.com/glfw/glfw.git
            # GLFW 3.4 release commit.
            GIT_TAG 7b6aead9fb88b3623e3b3725ebb42670cbe4c579
        )
        FetchContent_MakeAvailable(glfw)
        set(PHYSICS_GLFW_TARGET glfw)
    else()
        message(FATAL_ERROR "GLFW 3.3+ not found. Set CMAKE_PREFIX_PATH to its installation or enable PHYSICS_FETCH_GLFW.")
    endif()
    add_executable(physics_viewer viewer.cpp)
    target_link_libraries(physics_viewer PRIVATE physics_app ${PHYSICS_GLFW_TARGET} OpenGL::GL)
    # Keep warnings local to project targets; do not alter GLFW's compiler flags.
    physics_project_warnings(physics_viewer)
endif()
