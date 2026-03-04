/**
 * imu_buffer.cpp
 *
 * Implementation of the fixed-capacity circular buffer for IMU readings.
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

#include "imu_buffer.hpp"

/* =========================================================================
 * IMUCircularBuffer
 * ====================================================================== */

IMUCircularBuffer::IMUCircularBuffer()
    : head_(0), count_(0)
{
    // buf_ is zero-initialised by the default constructor of each IMUReading
    // (plain struct, all members are floats — zero-inited in .bss).
}

void IMUCircularBuffer::push(const IMUReading &reading)
{
    buf_[head_] = reading;
    head_ = (head_ + 1) % IMU_BUFFER_CAPACITY;

    if (count_ < IMU_BUFFER_CAPACITY) {
        ++count_;
    }
    // When full: head_ just overwrote the oldest slot and advanced past it,
    // which is correct — head_ now points to the next-oldest slot (the new
    // "oldest" entry), maintaining the invariant.
}

const IMUReading *IMUCircularBuffer::latest() const
{
    if (count_ == 0) return nullptr;

    // The most recent write was at (head_ - 1 + capacity) % capacity.
    size_t last = (head_ + IMU_BUFFER_CAPACITY - 1) % IMU_BUFFER_CAPACITY;
    return &buf_[last];
}

size_t IMUCircularBuffer::get_all(IMUReading *out) const
{
    if (count_ == 0) return 0;

    // Oldest entry is at (head_ - count_ + capacity) % capacity.
    size_t start = (head_ + IMU_BUFFER_CAPACITY - count_) % IMU_BUFFER_CAPACITY;

    for (size_t i = 0; i < count_; ++i) {
        out[i] = buf_[(start + i) % IMU_BUFFER_CAPACITY];
    }
    return count_;
}

/* =========================================================================
 * Global instance + C shim
 * ====================================================================== */

// Single global instance — lives in .bss, constructed before main() by the
// C++ startup code (startup_stm32f4xx.s calls __libc_init_array).
static IMUCircularBuffer g_imu_buf;

extern "C" {

void imu_buffer_push(float ax, float ay, float az,
                     float gx, float gy, float gz)
{
    IMUReading r;
    r.ax = ax; r.ay = ay; r.az = az;
    r.gx = gx; r.gy = gy; r.gz = gz;
    g_imu_buf.push(r);
}

int imu_buffer_get_latest(IMUReading *out)
{
    const IMUReading *p = g_imu_buf.latest();
    if (!p) return 0;
    *out = *p;
    return 1;
}

size_t imu_buffer_get_all(IMUReading *out)
{
    return g_imu_buf.get_all(out);
}

size_t imu_buffer_count(void)
{
    return g_imu_buf.size();
}

} // extern "C"
