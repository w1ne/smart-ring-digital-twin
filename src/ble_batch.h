/*
 * ble_batch.h - packs sensor records into GATT notification payloads.
 *
 * This is the concrete form of the batching argument in the architecture
 * doc. The bandwidth is trivial (50 Hz x 12 bytes ~= 600 B/s); the cost is
 * radio *wakeups*. One notification per sample means 50 radio events per
 * second. Packing 16 samples per notification cuts that to ~3/s for the same
 * data, and radio wakeups are what the battery actually pays for.
 *
 * Wire format (little-endian, fits one ATT_MTU=247 notification)
 * -------------------------------------------------------------
 *   off  size  field
 *   0    1     type      0x01 IMU batch, 0x02 temperature batch
 *   1    1     count     records in this frame
 *   2    4     first_seq sequence number of record 0
 *   6    8     base_t_us absolute timestamp of record 0
 *   14   2     dropped   lifetime dropped count, saturating at 0xFFFF
 *   16   ...   records
 *
 * IMU record (14 B):   dt_us:u16, accel[3]:i16, gyro[3]:i16
 * Temp record (6 B):   dt_ms:u16, milli_celsius:i32
 *
 * dt is the gap from the PREVIOUS record, not the offset from base_t_us, and
 * is zero for record 0. The phone reconstructs absolute time by accumulating.
 * Only one absolute timestamp is sent per frame instead of eight bytes per
 * sample.
 *
 * Per-sample rather than cumulative because cumulative overflows almost
 * immediately: a u16 of microseconds spans 65.5 ms, which at 50 Hz is three
 * samples. Encoding the gap instead means the field only has to hold one
 * sample period, so the batch size is limited by the ATT payload - which is
 * the thing we actually want to fill - rather than by the timestamp field.
 *
 * The unit still differs per stream, chosen from that stream's rate:
 *
 *   IMU  at 50 Hz -> 20 ms gaps -> dt in microseconds, u16 tolerates a 65 ms stall
 *   Temp at  1 Hz -> 1 s gaps   -> dt in milliseconds, u16 tolerates a 65 s stall
 *
 * A gap too large for the field closes the batch early rather than emitting a
 * wrong delta; the remaining samples stay buffered for the next frame. After
 * this change that is a genuine stall, not routine arithmetic overflow.
 *
 * first_seq + count + dropped is what lets the phone detect loss precisely:
 * a gap between the last frame's last sequence number and this frame's
 * first_seq is exactly the number of samples that were never delivered.
 */
#ifndef BLE_BATCH_H
#define BLE_BATCH_H

#include <stddef.h>
#include <stdint.h>

#include "sensor_manager.h"

#define BLE_BATCH_TYPE_IMU  0x01u
#define BLE_BATCH_TYPE_TEMP 0x02u

#define BLE_BATCH_HDR_LEN 16u
#define BLE_IMU_REC_LEN   14u
#define BLE_TEMP_REC_LEN  6u

/* The header's `count` field is one byte, so a single frame can describe at
 * most this many records regardless of how large the payload budget is. */
#define BLE_BATCH_MAX_RECORDS 255u

/* Default ATT payload budget: ATT_MTU 247 minus the 3-byte notification
 * header. Passed in explicitly so the caller can use the *negotiated* MTU
 * rather than an assumed one. */
#define BLE_DEFAULT_PAYLOAD 244u

/* Maximum records that fit a given payload budget. */
#define BLE_IMU_MAX_RECORDS(payload)  (((payload) - BLE_BATCH_HDR_LEN) / BLE_IMU_REC_LEN)
#define BLE_TEMP_MAX_RECORDS(payload) (((payload) - BLE_BATCH_HDR_LEN) / BLE_TEMP_REC_LEN)

/*
 * Encode up to `n` samples into `out` (capacity `out_cap` bytes).
 *
 * Returns the number of bytes written, or 0 if nothing could be encoded.
 * `consumed` receives the number of input samples actually packed, which may
 * be fewer than `n` when the payload budget or the 65.5 ms delta window runs
 * out first. The caller releases exactly `*consumed` samples from the ring
 * buffer, so a partially-packed batch never loses data.
 */
size_t ble_batch_encode_imu(const sm_imu_sample_t *samples, size_t n, uint32_t dropped,
                            uint8_t *out, size_t out_cap, size_t *consumed);

size_t ble_batch_encode_temp(const sm_temp_sample_t *samples, size_t n, uint32_t dropped,
                             uint8_t *out, size_t out_cap, size_t *consumed);

/* Header accessors, used by the tests to verify framing without duplicating
 * the offset arithmetic under test. */
uint8_t  ble_batch_type(const uint8_t *frame);
uint8_t  ble_batch_count(const uint8_t *frame);
uint32_t ble_batch_first_seq(const uint8_t *frame);
uint64_t ble_batch_base_time(const uint8_t *frame);
uint16_t ble_batch_dropped(const uint8_t *frame);

#endif /* BLE_BATCH_H */
