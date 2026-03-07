/**
 * imu_buffer.h
 *
 * C-compatible interface for the sensor data circular buffer.
 * Each entry contains a timestamp, motor position, and IMU readings.
 */

#ifndef INC_IMU_BUFFER_H_
#define INC_IMU_BUFFER_H_

#include <stddef.h>
#include <stdint.h>

#define IMU_BUFFER_CAPACITY 100

typedef struct {
    uint32_t timestamp_ms;    /* HAL_GetTick() at time of capture */
    float motor_position_deg; /* Motor position in degrees (from CAN feedback) */
    float ax, ay, az;         /* Accelerometer (m/s^2) */
    float gx, gy, gz;        /* Gyroscope (dps) */
} SensorReading;

/* Legacy name kept for minimal churn in existing code */
typedef SensorReading IMUReading;

#ifdef __cplusplus
extern "C" {
#endif

/** Push one sensor reading into the global ring buffer. */
void imu_buffer_push(uint32_t timestamp_ms, float motor_pos_deg,
                     float ax, float ay, float az,
                     float gx, float gy, float gz);

/**
 * Copy the most recent reading into *out.
 * Returns 1 on success, 0 if the buffer is empty.
 */
int imu_buffer_get_latest(SensorReading *out);

/**
 * Copy all stored readings (oldest first) into the array pointed to by out.
 * Returns the number of entries written (0 if empty).
 * out must hold at least IMU_BUFFER_CAPACITY elements.
 */
size_t imu_buffer_get_all(SensorReading *out);

/** Returns the number of valid readings currently stored. */
size_t imu_buffer_count(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_BUFFER_H_ */
