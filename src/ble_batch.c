/*
 * ble_batch.c - little-endian encoder for the GATT notification frames.
 *
 * The wire format and the reasoning behind delta encoding, per-stream delta
 * units, saturation and early batch closure live in ble_batch.h. This file is
 * the serialiser: explicit byte writers (no struct-over-the-air), the header
 * layout as named field offsets so the writer and the reader accessors cannot
 * drift, and two encoders that share their record-budget arithmetic.
 */
#include "ble_batch.h"

#include <stdint.h>
#include <string.h>

/*
 * Header field byte offsets (see the wire-format table in ble_batch.h). Named
 * here so write_header() and the ble_batch_*() accessors reference one source
 * of truth instead of repeating literal offsets that must stay in lockstep.
 */
#define BLE_OFF_TYPE      0u
#define BLE_OFF_COUNT     1u
#define BLE_OFF_FIRST_SEQ 2u
#define BLE_OFF_BASE_T    6u
#define BLE_OFF_DROPPED   14u

/* Microseconds per millisecond: the temperature stream reports deltas in ms. */
#define BLE_US_PER_MS 1000ull

/* Explicit little-endian writers. Never memcpy a struct onto the air: the
 * compiler's padding and the host's endianness are not part of the protocol
 * contract, and the phone team does not want to discover that. */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void put_u64(uint8_t *p, uint64_t v)
{
    put_u32(p, (uint32_t)(v & 0xFFFFFFFFu));
    put_u32(p + 4, (uint32_t)(v >> 32));
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void write_header(uint8_t *out, uint8_t type, uint8_t count, uint32_t first_seq,
                         uint64_t base_t_us, uint32_t dropped)
{
    out[BLE_OFF_TYPE]  = type;
    out[BLE_OFF_COUNT] = count;
    put_u32(out + BLE_OFF_FIRST_SEQ, first_seq);
    put_u64(out + BLE_OFF_BASE_T, base_t_us);
    /* Saturate rather than wrap: "at least 65535 lost" is a true statement,
     * a wrapped small number is a lie. */
    put_u16(out + BLE_OFF_DROPPED, dropped > UINT16_MAX ? UINT16_MAX : (uint16_t)dropped);
}

/*
 * Records of `rec_len` bytes that fit `out_cap`, then clamped to the `n`
 * available and to what the one-byte count field can describe. Shared by both
 * encoders so the payload-budget arithmetic lives in exactly one place.
 * Precondition: out_cap >= BLE_BATCH_HDR_LEN + rec_len (callers check).
 */
static size_t batch_max_records(size_t out_cap, size_t rec_len, size_t n)
{
    size_t fit = (out_cap - BLE_BATCH_HDR_LEN) / rec_len;

    if (fit > n) {
        fit = n;
    }
    if (fit > BLE_BATCH_MAX_RECORDS) {
        fit = BLE_BATCH_MAX_RECORDS;
    }
    return fit;
}

size_t ble_batch_encode_imu(const sm_imu_sample_t *samples, size_t n, uint32_t dropped,
                            uint8_t *out, size_t out_cap, size_t *consumed)
{
    size_t   max_rec, i, off;
    uint64_t base;

    *consumed = 0;
    if (samples == NULL || out == NULL || n == 0 ||
        out_cap < BLE_BATCH_HDR_LEN + BLE_IMU_REC_LEN) {
        return 0;
    }

    max_rec = batch_max_records(out_cap, BLE_IMU_REC_LEN, n);

    base = samples[0].t_us;
    off  = BLE_BATCH_HDR_LEN;

    for (i = 0; i < max_rec; i++) {
        /* Gap from the previous sample, not from the base. See ble_batch.h. */
        uint64_t delta = (i == 0) ? 0u : (samples[i].t_us - samples[i - 1].t_us);
        int      j;

        /* Close the batch rather than emit a truncated delta. The remaining
         * samples stay in the buffer and go out in the next frame. */
        if (delta > UINT16_MAX) {
            break;
        }

        put_u16(out + off, (uint16_t)delta);
        off += 2;
        for (j = 0; j < SM_IMU_AXES; j++) {
            put_u16(out + off, (uint16_t)samples[i].accel[j]);
            off += 2;
        }
        for (j = 0; j < SM_IMU_AXES; j++) {
            put_u16(out + off, (uint16_t)samples[i].gyro[j]);
            off += 2;
        }
    }

    if (i == 0) {
        return 0;
    }

    write_header(out, BLE_BATCH_TYPE_IMU, (uint8_t)i, samples[0].seq, base, dropped);
    *consumed = i;
    return off;
}

size_t ble_batch_encode_temp(const sm_temp_sample_t *samples, size_t n, uint32_t dropped,
                             uint8_t *out, size_t out_cap, size_t *consumed)
{
    size_t   max_rec, i, off;
    uint64_t base;

    *consumed = 0;
    if (samples == NULL || out == NULL || n == 0 ||
        out_cap < BLE_BATCH_HDR_LEN + BLE_TEMP_REC_LEN) {
        return 0;
    }

    max_rec = batch_max_records(out_cap, BLE_TEMP_REC_LEN, n);

    base = samples[0].t_us;
    off  = BLE_BATCH_HDR_LEN;

    for (i = 0; i < max_rec; i++) {
        /* Gap from the previous sample, in milliseconds: at 1 Hz a
         * microsecond field would not even hold one period. See ble_batch.h. */
        uint64_t delta_ms =
            (i == 0) ? 0u : (samples[i].t_us - samples[i - 1].t_us) / BLE_US_PER_MS;

        if (delta_ms > UINT16_MAX) {
            break;
        }
        put_u16(out + off, (uint16_t)delta_ms);
        off += 2;
        put_u32(out + off, (uint32_t)samples[i].milli_celsius);
        off += 4;
    }

    if (i == 0) {
        return 0;
    }

    write_header(out, BLE_BATCH_TYPE_TEMP, (uint8_t)i, samples[0].seq, base, dropped);
    *consumed = i;
    return off;
}

uint8_t ble_batch_type(const uint8_t *frame)
{
    return frame[BLE_OFF_TYPE];
}
uint8_t ble_batch_count(const uint8_t *frame)
{
    return frame[BLE_OFF_COUNT];
}
uint32_t ble_batch_first_seq(const uint8_t *frame)
{
    return get_u32(frame + BLE_OFF_FIRST_SEQ);
}
uint16_t ble_batch_dropped(const uint8_t *frame)
{
    return get_u16(frame + BLE_OFF_DROPPED);
}

uint64_t ble_batch_base_time(const uint8_t *frame)
{
    return (uint64_t)get_u32(frame + BLE_OFF_BASE_T) |
           ((uint64_t)get_u32(frame + BLE_OFF_BASE_T + 4) << 32);
}
