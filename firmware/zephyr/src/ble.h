/*
 * ble.h - Ring Data Service: a minimal BLE peripheral that streams IMU batches.
 *
 * Built only when CONFIG_BT is enabled (see ble.conf); otherwise the two
 * entry points below compile to no-ops so main.c needs no #ifdefs. BLE is a
 * DK-only feature: LabWired does not model the nRF54L radio, so this path is
 * exercised on real silicon, not in the simulator.
 */
#ifndef BLE_H
#define BLE_H

#include "sensor_manager.h"

/* Start advertising the Ring Data Service. No-op (returns 0) without CONFIG_BT. */
int ble_start(void);

/*
 * Drain pending IMU samples from the manager into one GATT notification,
 * encoded with ble_batch_encode_imu. Does nothing if no central is connected
 * or nothing is buffered. No-op without CONFIG_BT.
 */
void ble_notify_imu(sensor_manager_t *mgr);

#endif /* BLE_H */
