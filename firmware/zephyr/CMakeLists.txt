# Smart-ring firmware for the Nordic nRF54L15 DK (cpuapp).
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smart_ring)

# Portable sensor-manager core, compiled unmodified into the app, plus its
# Zephyr port. SM_PORT_ZEPHYR selects the k_spinlock / k_uptime backing.
set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
  src/main.c
  src/sensors.c
  src/ble.c
  ${CORE_DIR}/sensor_manager.c
  ${CORE_DIR}/sm_ringbuf.c
  ${CORE_DIR}/port/sm_port_zephyr.c
)

# ble_batch is only needed by the BLE notify path (enabled via ble.conf).
if(CONFIG_BT)
  target_sources(app PRIVATE ${CORE_DIR}/ble_batch.c)
endif()

target_include_directories(app PRIVATE ${CORE_DIR})
target_compile_definitions(app PRIVATE SM_PORT_ZEPHYR)
