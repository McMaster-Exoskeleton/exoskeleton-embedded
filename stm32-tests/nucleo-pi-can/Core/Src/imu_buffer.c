/**
 * imu_buffer.c
 *
 * Pure-C implementation of the fixed-capacity circular buffer for sensor readings.
 * Each entry now includes a timestamp (ms) and motor position (degrees) alongside
 * the six IMU values.
 *
 * Memory: 100 * sizeof(SensorReading) = 100 * 32 bytes = 3200 bytes in .bss
 */

#include "imu_buffer.h"

typedef struct {
    SensorReading buf[IMU_BUFFER_CAPACITY];
    size_t        head;
    size_t        count;
} SensorCircularBuffer;

static SensorCircularBuffer g_buf;  /* zero-initialised in .bss */

void imu_buffer_push(uint32_t timestamp_ms, float motor_pos_deg,
                     float ax, float ay, float az,
                     float gx, float gy, float gz)
{
    SensorReading *slot = &g_buf.buf[g_buf.head];
    slot->timestamp_ms    = timestamp_ms;
    slot->motor_position_deg = motor_pos_deg;
    slot->ax = ax; slot->ay = ay; slot->az = az;
    slot->gx = gx; slot->gy = gy; slot->gz = gz;

    g_buf.head = (g_buf.head + 1) % IMU_BUFFER_CAPACITY;

    if (g_buf.count < IMU_BUFFER_CAPACITY) {
        ++g_buf.count;
    }
}

int imu_buffer_get_latest(SensorReading *out)
{
    if (g_buf.count == 0) return 0;

    size_t last = (g_buf.head + IMU_BUFFER_CAPACITY - 1) % IMU_BUFFER_CAPACITY;
    *out = g_buf.buf[last];
    return 1;
}

size_t imu_buffer_get_all(SensorReading *out)
{
    if (g_buf.count == 0) return 0;

    size_t start = (g_buf.head + IMU_BUFFER_CAPACITY - g_buf.count) % IMU_BUFFER_CAPACITY;

    for (size_t i = 0; i < g_buf.count; ++i) {
        out[i] = g_buf.buf[(start + i) % IMU_BUFFER_CAPACITY];
    }
    return g_buf.count;
}

size_t imu_buffer_count(void)
{
    return g_buf.count;
}
