/*
 * ble.c - Ring Data Service (IMU batch notifications). See ble.h.
 */
#include "ble.h"

#if defined(CONFIG_BT)

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>

#include "ble_batch.h"
#include "sensor_manager.h"

/* Ring Data Service, random 128-bit base:
 *   service        7e9d0001-... , IMU-batch characteristic 7e9d0002-...
 */
#define RDS_UUID_SERVICE                                                                       \
	BT_UUID_128_ENCODE(0x7e9d0001, 0xc4b3, 0x4f2a, 0x9a10, 0x0011223344ff)
#define RDS_UUID_IMU_BATCH                                                                     \
	BT_UUID_128_ENCODE(0x7e9d0002, 0xc4b3, 0x4f2a, 0x9a10, 0x0011223344ff)

static const struct bt_uuid_128 rds_service = BT_UUID_INIT_128(RDS_UUID_SERVICE);
static const struct bt_uuid_128 rds_imu_batch = BT_UUID_INIT_128(RDS_UUID_IMU_BATCH);

static struct bt_conn *current_conn;
static bool notify_enabled;

static void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

/* Notify-only characteristic: the phone subscribes, the ring pushes batches. */
BT_GATT_SERVICE_DEFINE(rds_svc,
	BT_GATT_PRIMARY_SERVICE(&rds_service),
	BT_GATT_CHARACTERISTIC(&rds_imu_batch.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(imu_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

/* The notify attribute is the characteristic value, index 2 in the table. */
#define IMU_VALUE_ATTR (&rds_svc.attrs[2])

static const struct bt_data adv[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err == 0) {
		current_conn = bt_conn_ref(conn);
	}
	printk("ble: connected (err %u)\n", err);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	notify_enabled = false;
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	printk("ble: disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
};

int ble_start(void)
{
	int err = bt_enable(NULL);

	if (err) {
		printk("ble: bt_enable failed (%d)\n", err);
		return err;
	}
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, adv, ARRAY_SIZE(adv), NULL, 0);
	if (err) {
		printk("ble: adv start failed (%d)\n", err);
	}
	return err;
}

void ble_notify_imu(sensor_manager_t *mgr)
{
	if (!current_conn || !notify_enabled) {
		return;
	}

	sm_imu_sample_t samples[BLE_IMU_MAX_RECORDS(BLE_DEFAULT_PAYLOAD)];
	size_t n = sm_peek_imu(mgr, samples, ARRAY_SIZE(samples));

	if (n == 0) {
		return;
	}

	sm_stats_t stats;

	sm_get_stats(mgr, &stats);

	uint8_t frame[BLE_DEFAULT_PAYLOAD];
	size_t consumed = 0;
	size_t len =
		ble_batch_encode_imu(samples, n, stats.imu.dropped, frame, sizeof(frame), &consumed);

	if (len > 0 && bt_gatt_notify(current_conn, IMU_VALUE_ATTR, frame, len) == 0) {
		sm_release_imu(mgr, consumed);
	}
}

#else /* !CONFIG_BT: inert stubs so main.c needs no #ifdefs. */

int ble_start(void)
{
	return 0;
}

void ble_notify_imu(sensor_manager_t *mgr)
{
	(void)mgr;
}

#endif /* CONFIG_BT */
