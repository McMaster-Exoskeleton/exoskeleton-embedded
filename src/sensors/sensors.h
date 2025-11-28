/**
 * @file sensors.h
 * @brief Sensor interface class for exoskeleton embedded system
 * 
 * This class provides an interface for reading and managing various sensors
 * used in the exoskeleton system (e.g., IMU, encoders, force sensors, etc.)
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @class Sensors
 * @brief Main sensor management class for the exoskeleton
 */
class Sensors {
public:
    /**
     * @brief Constructor
     */
    Sensors();

    /**
     * @brief Destructor
     */
    ~Sensors();

    /**
     * @brief Initialize all sensors
     * @return true if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * @brief Deinitialize all sensors
     */
    void deinitialize();

    /**
     * @brief Read sensor data
     * @return true if read successful, false otherwise
     */
    bool read();

    /**
     * @brief Check if sensors are initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;

    // TODO: Add specific sensor reading methods
    // Example:
    // float getIMUAccelerationX() const;
    // float getIMUAccelerationY() const;
    // float getIMUAccelerationZ() const;
    // float getEncoderPosition(uint8_t motor_id) const;
    // float getForceSensorReading(uint8_t sensor_id) const;

private:
    bool initialized_;  ///< Initialization status flag

    // TODO: Add sensor-specific data members
    // Example:
    // IMUData imu_data_;
    // EncoderData encoder_data_[MAX_MOTORS];
    // ForceSensorData force_data_[MAX_FORCE_SENSORS];
};

#endif // SENSORS_H

