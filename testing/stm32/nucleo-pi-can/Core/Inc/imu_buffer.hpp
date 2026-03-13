/**
 * imu_buffer.hpp
 *
 * C++ class definition for the IMU circular buffer.
 * Only include this from .cpp files. C files should include imu_buffer.h.
 *
 * Design rationale:
 *   - Capacity is fixed at compile time (IMU_BUFFER_CAPACITY = 100). A static
 *     array avoids heap allocation, which is hazardous in bare-metal firmware.
 *   - The overwrite-on-full policy (oldest sample discarded when the buffer is
 *     full) is enforced inside push(), keeping the implementation lock-step
 *     with the DMA callback without requiring caller coordination.
 *   - All public methods are O(1) except get_all(), which is O(N).
 *
 * Thread / ISR safety:
 *   The DMA callback (HAL_I2C_MemRxCpltCallback) runs from an ISR and calls
 *   imu_buffer_push(). main() reads via imu_buffer_get_latest() and
 *   imu_buffer_get_all(). To prevent torn reads, callers in main() must
 *   disable/re-enable the DMA interrupt around any multi-step access — this is
 *   handled in process_command() in main.c using HAL_NVIC_Disable/EnableIRQ.
 */

#pragma once

#include "imu_buffer.h"

class IMUCircularBuffer {
public:
    IMUCircularBuffer();

    void push(const IMUReading &reading);
    const IMUReading *latest() const;
    size_t get_all(IMUReading *out) const;

    size_t size() const { return count_; }
    bool   empty() const { return count_ == 0; }
    bool   full()  const { return count_ == IMU_BUFFER_CAPACITY; }

private:
    IMUReading buf_[IMU_BUFFER_CAPACITY];
    size_t     head_;
    size_t     count_;
};
