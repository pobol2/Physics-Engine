set(PHYSICS_TEST_OUTPUT "${PROJECT_BINARY_DIR}/test-output")

function(physics_test_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE physics::engine physics_examples)
    physics_project_warnings(${name})
endfunction()

physics_test_executable(physics_tests physics_tests.cpp)
foreach(test_name IN ITEMS forces_energy initialization oscillator_accuracy oscillator_convergence
                           pendulum_small_angle pendulum_nonlinear pendulum_period
                           conservation validation simulation_time mass_gravity spring_rest_length
                           linear_damping fixed_position constraint_edge_cases damped_examples)
    add_test(NAME ${test_name} COMMAND physics_tests ${test_name})
endforeach()

physics_test_executable(physics_numerics numerical_validation.cpp)
add_test(NAME numerical_accuracy COMMAND physics_numerics
    "${PHYSICS_TEST_OUTPUT}/numerical-validation.csv")

physics_test_executable(physics_dynamics_numerics dynamics_validation.cpp)
add_test(NAME dynamics_accuracy COMMAND physics_dynamics_numerics
    "${PHYSICS_TEST_OUTPUT}/dynamics-validation.csv")

physics_test_executable(physics_library_tests library_tests.cpp)
foreach(test_name IN ITEMS motions force_library external_power coupled_constraints custom_constraints
                           adaptive_integration diagnostics example_catalog)
    add_test(NAME ${test_name} COMMAND physics_library_tests ${test_name})
endforeach()

physics_test_executable(physics_example_tests example_tests.cpp)
foreach(test_name IN ITEMS driven_oscillator parametric_oscillator kapitza_pendulum double_pendulum
                           rotating_hoop kepler three_body cyclotron)
    add_test(NAME example_${test_name} COMMAND physics_example_tests ${test_name})
endforeach()

foreach(test_name IN ITEMS cli_csv cli_errors)
    add_test(NAME ${test_name} COMMAND "${CMAKE_COMMAND}"
        "-DRUNNER=$<TARGET_FILE:physics_runner>"
        "-DOUTPUT_DIR=${PHYSICS_TEST_OUTPUT}/${test_name}"
        "-DCASE=${test_name}" -P "${CMAKE_CURRENT_SOURCE_DIR}/cli_test.cmake")
endforeach()

if(PHYSICS_BUILD_VIEWER)
    # Exercises shared configuration without initializing graphics.
    add_test(NAME viewer_help COMMAND physics_viewer --help)
endif()

# Builds examples/external-project as a separate CMake project that consumes only
# engine/ through add_subdirectory, then runs it. Eigen is handed down so the
# nested configure needs no network access.
set(consumer_options
    "-DPHYSICS_ENGINE_DIR=${PROJECT_SOURCE_DIR}/engine"
    "-DCMAKE_BUILD_TYPE=$<CONFIG>"
    "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
if(NOT CMAKE_GENERATOR MATCHES "Visual Studio")
    list(APPEND consumer_options "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
endif()
include(FetchContent)
FetchContent_GetProperties(eigen)
if(eigen_SOURCE_DIR)
    list(APPEND consumer_options "-DFETCHCONTENT_SOURCE_DIR_EIGEN=${eigen_SOURCE_DIR}")
elseif(Eigen3_DIR)
    list(APPEND consumer_options "-DEigen3_DIR=${Eigen3_DIR}")
endif()
set(consumer_generator_options --build-generator "${CMAKE_GENERATOR}")
if(CMAKE_GENERATOR_PLATFORM)
    list(APPEND consumer_generator_options --build-generator-platform "${CMAKE_GENERATOR_PLATFORM}")
endif()
if(CMAKE_MAKE_PROGRAM)
    list(APPEND consumer_generator_options --build-makeprogram "${CMAKE_MAKE_PROGRAM}")
endif()
add_test(NAME engine_consumer COMMAND "${CMAKE_CTEST_COMMAND}"
    --build-and-test "${PROJECT_SOURCE_DIR}/examples/external-project"
                     "${CMAKE_CURRENT_BINARY_DIR}/external-project"
    ${consumer_generator_options}
    --build-config $<CONFIG>
    --build-noclean
    --build-options ${consumer_options}
    --test-command pendulum_study "${CMAKE_CURRENT_BINARY_DIR}/external-project/pendulum.csv")
