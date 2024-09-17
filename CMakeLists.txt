cmake_minimum_required(VERSION 3.13.0)

set(PHYSICS_PROJECT_NAME
    physics
)
set(PHYSICS_SOURCE_DIR
    src
)
set(PHYSICS_SOURCE
    ${PHYSICS_SOURCE_DIR}/Vector.cpp
    ${PHYSICS_SOURCE_DIR}/Particle.cpp
    ${PHYSICS_SOURCE_DIR}/Matrix.cpp
    ${PHYSICS_SOURCE_DIR}/Identity.cpp
    ${PHYSICS_SOURCE_DIR}/System.cpp
    ${PHYSICS_SOURCE_DIR}/main.cpp
)
set(PHYSICS_LIBRARIES_DIR
    libs
)
set(PHYSICS_LIBRARIES
)

# Specify include directories
include_directories(include)

project(${PHYSICS_PROJECT_NAME})

add_executable(${PHYSICS_PROJECT_NAME} ${PHYSICS_SOURCE})

foreach(LIBRARY ${PHYSICS_LIBRARIES})
    add_subdirectory("${PHYSICS_LIBRARIES_DIR}/${LIBRARY}")
endforeach(LIBRARY)
target_link_libraries(${PHYSICS_PROJECT_NAME} ${PHYSICS_LIBRARIES})