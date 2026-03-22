/**
 * imu_buffer.c
 *
 * Pure-C implementation of the fixed-capacity circular buffer for IMU readings.
 *
 * Data structure: circular array (ring buffer) with a head pointer and a count.
 *
 *   head_  — always points to the slot where the *next* write will go.
 *   count_ — tracks how many valid entries exist (saturates at capacity).
 *
 * On push when full:
 *   Writing at head_ overwrites the oldest entry (which is at head_ when
 *   the buffer is full, because head_ has wrapped around to the oldest slot).
 *   head_ then advances, so the invariant is maintained without a separate
 *   tail pointer.
 *
 * On get_all:
 *   The oldest entry is at index (head_ - count_ + capacity) % capacity.
 *   We iterate count_ steps from there, wrapping with modulo, copying into
 *   the output array in chronological order.
 *
 * Memory: 100 * sizeof(IMUReading) = 100 * 24 bytes = 2400 bytes in .bss
 *         (all floats, zero-initialised by the C runtime).
 */
#include "imu_buffer.h"

typedef struct {
    IMUReading buf[IMU_BUFFER_CAPACITY];
    size_t     head;
    size_t     count;
} IMUCircularBuffer;

static IMUCircularBuffer g_imu_buf;  /* zero-initialised in .bss */

void imu_buffer_push(uint32_t tick, float ax, float ay, float az,
                     float gx, float gy, float gz)
{
    IMUReading *slot = &g_imu_buf.buf[g_imu_buf.head];

    slot->tick = tick;
    slot->ax = ax; slot->ay = ay; slot->az = az;
    slot->gx = gx; slot->gy = gy; slot->gz = gz;

    g_imu_buf.head = (g_imu_buf.head + 1) % IMU_BUFFER_CAPACITY;

    if (g_imu_buf.count < IMU_BUFFER_CAPACITY) {
        ++g_imu_buf.count;
    }
}

int imu_buffer_get_latest(IMUReading *out)
{
    if (g_imu_buf.count == 0) return 0;

    size_t last = (g_imu_buf.head + IMU_BUFFER_CAPACITY - 1) % IMU_BUFFER_CAPACITY;
    *out = g_imu_buf.buf[last];
    return 1;
}

size_t imu_buffer_get_all(IMUReading *out)
{
    if (g_imu_buf.count == 0) return 0;

    size_t start = (g_imu_buf.head + IMU_BUFFER_CAPACITY - g_imu_buf.count) % IMU_BUFFER_CAPACITY;

    for (size_t i = 0; i < g_imu_buf.count; ++i) {
        out[i] = g_imu_buf.buf[(start + i) % IMU_BUFFER_CAPACITY];
    }
    return g_imu_buf.count;
}

size_t imu_buffer_count(void)
{
    return g_imu_buf.count;
}
