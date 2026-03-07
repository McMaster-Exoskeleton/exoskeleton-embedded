/**
 * imu_buffer.h
 *
 * C-compatible interface for the IMU circular buffer.
 * Include this from .c files. The C++ implementation is in imu_buffer.cpp.
 */

#ifndef INC_IMU_BUFFER_H_
#define INC_IMU_BUFFER_H_

#include <stddef.h>

#define IMU_BUFFER_CAPACITY 100

typedef struct {
    float ax, ay, az;   /* Accelerometer (m/s^2) */
    float gx, gy, gz;   /* Gyroscope (dps) */
} IMUReading;

#ifdef __cplusplus
extern "C" {
#endif

/** Push one IMU reading into the global ring buffer. */
void imu_buffer_push(float ax, float ay, float az,
                     float gx, float gy, float gz);

/**
 * Copy the most recent reading into *out.
 * Returns 1 on success, 0 if the buffer is empty.
 */
int imu_buffer_get_latest(IMUReading *out);

/**
 * Copy all stored readings (oldest first) into the array pointed to by out.
 * Returns the number of entries written (0 if empty).
 * out must hold at least IMU_BUFFER_CAPACITY elements.
 */
size_t imu_buffer_get_all(IMUReading *out);

/** Returns the number of valid readings currently stored. */
size_t imu_buffer_count(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_BUFFER_H_ */
