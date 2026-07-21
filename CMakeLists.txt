cmake_minimum_required(VERSION 3.16)
project(smart_ring_sensor_manager C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warnings as errors. On a wearable, a warning you shipped is a field return
# you have not had yet.
add_compile_options(
    -Wall -Wextra -Wpedantic -Werror
    -Wshadow -Wconversion -Wsign-conversion
    -Wcast-align -Wstrict-prototypes -Wmissing-prototypes
    -Wpointer-arith -Wwrite-strings
)

set(CORE_SOURCES
    src/sm_ringbuf.c
    src/sensor_manager.c
    src/sensor_sim.c
    src/ble_batch.c
)

# --------------------------------------------------------------------------
# The core is built twice, against two different ports. Same sources, no
# ifdefs in the core itself - that is the whole point of the port layer, and
# building both proves it rather than claiming it.
#
#   sm_core_none  : single-threaded, virtual time. Deterministic logic tests.
#   sm_core_posix : pthreads, real monotonic clock. Threading and the demo.
#
# src/port/sm_port_zephyr.c is deliberately NOT built here: it needs a Zephyr
# tree and target hardware. It is included as a design artefact only.
# --------------------------------------------------------------------------

add_library(sm_core_none STATIC ${CORE_SOURCES} src/port/sm_port_none.c)
target_compile_definitions(sm_core_none PUBLIC SM_PORT_NONE)
target_include_directories(sm_core_none PUBLIC src)

find_package(Threads REQUIRED)

add_library(sm_core_posix STATIC ${CORE_SOURCES} src/port/sm_port_posix.c)
target_compile_definitions(sm_core_posix PUBLIC SM_PORT_POSIX)
target_include_directories(sm_core_posix PUBLIC src)
target_link_libraries(sm_core_posix PUBLIC Threads::Threads)

# --------------------------------------------------------------------------
# Demo
# --------------------------------------------------------------------------

add_executable(sensor_demo app/demo_main.c)
target_link_libraries(sensor_demo PRIVATE sm_core_posix)

# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------

enable_testing()

function(add_sm_test name source lib)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ${lib})
    target_include_directories(${name} PRIVATE tests)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_sm_test(test_ringbuf        tests/test_ringbuf.c        sm_core_none)
add_sm_test(test_sensor_manager tests/test_sensor_manager.c sm_core_none)
add_sm_test(test_ble_batch      tests/test_ble_batch.c      sm_core_none)
add_sm_test(test_concurrency    tests/test_concurrency.c    sm_core_posix)

# Sanitizer variants. A concurrency test that has never run under TSan is a
# concurrency test that has not been run.
#
# Enabled with -DSM_SANITIZERS=ON. Off by default so a plain `cmake && ctest`
# works on any toolchain, including ones without sanitizer runtimes.
option(SM_SANITIZERS "Build address/UB and thread sanitizer test variants" OFF)

if(SM_SANITIZERS)
    # ASan+UBSan over the deterministic suites: catches the ring buffer
    # walking off its storage, which is the failure mode that would otherwise
    # show up as a silent corruption six months into a field trial.
    add_library(sm_core_none_asan STATIC ${CORE_SOURCES} src/port/sm_port_none.c)
    target_compile_definitions(sm_core_none_asan PUBLIC SM_PORT_NONE)
    target_include_directories(sm_core_none_asan PUBLIC src)
    target_compile_options(sm_core_none_asan PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer -g)
    target_link_options(sm_core_none_asan PUBLIC -fsanitize=address,undefined)

    add_sm_test(test_ringbuf_asan        tests/test_ringbuf.c        sm_core_none_asan)
    add_sm_test(test_sensor_manager_asan tests/test_sensor_manager.c sm_core_none_asan)
    add_sm_test(test_ble_batch_asan      tests/test_ble_batch.c      sm_core_none_asan)

    # TSan over the threaded suite.
    add_library(sm_core_posix_tsan STATIC ${CORE_SOURCES} src/port/sm_port_posix.c)
    target_compile_definitions(sm_core_posix_tsan PUBLIC SM_PORT_POSIX)
    target_include_directories(sm_core_posix_tsan PUBLIC src)
    target_compile_options(sm_core_posix_tsan PUBLIC -fsanitize=thread -fno-omit-frame-pointer -g)
    target_link_options(sm_core_posix_tsan PUBLIC -fsanitize=thread)
    target_link_libraries(sm_core_posix_tsan PUBLIC Threads::Threads)

    add_sm_test(test_concurrency_tsan tests/test_concurrency.c sm_core_posix_tsan)
endif()
