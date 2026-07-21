# Ztest build of the portable sensor-manager core.
#
# Same core sources as the host CMake/ctest build and the nRF54L15 app, linked
# UNMODIFIED. The only target-specific bit is SM_PORT_ZEPHYR, which selects the
# k_spinlock / k_uptime backing in src/port/sm_port_zephyr.c -- the exact port
# the real firmware runs. Runs on native_sim (host) and, via Twister, on target.
#
#   west build -b native_sim -t run tests/ztest
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_manager_ztest)

# The portable core lives two levels up, shared verbatim with the host tests
# and the firmware app. No copies, no forks: one source of truth.
set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
  # Ported Ztest suites (harness swapped, assertion intent preserved).
  src/test_ringbuf.c
  src/test_sensor_manager.c
  src/test_ble_batch.c
  src/test_concurrency.c
  # Unmodified portable core + its Zephyr port.
  ${CORE_DIR}/sm_ringbuf.c
  ${CORE_DIR}/sensor_manager.c
  ${CORE_DIR}/sensor_sim.c
  ${CORE_DIR}/ble_batch.c
  ${CORE_DIR}/port/sm_port_zephyr.c
)

target_include_directories(app PRIVATE ${CORE_DIR})
target_compile_definitions(app PRIVATE SM_PORT_ZEPHYR)
