# Ready-made example systems, built only from the engine's public API.
add_library(physics_examples STATIC
    catalog.cpp double_pendulum.cpp driven.cpp folded_chain.cpp orbits.cpp
    oscillator.cpp pendulum.cpp supports.cpp)
target_include_directories(physics_examples PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(physics_examples PUBLIC physics::engine)
physics_project_warnings(physics_examples)
