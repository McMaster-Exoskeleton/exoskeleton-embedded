/**
 * @file motors.h
 * @brief Motor control class for exoskeleton embedded system
 * 
 * This class provides an interface for controlling motors in the exoskeleton
 * system, including position, velocity, and torque control modes.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @enum MotorControlMode
 * @brief Motor control mode enumeration
 */
enum class MotorControlMode {
    DISABLED,      ///< Motor disabled
    POSITION,      ///< Position control mode
    VELOCITY,      ///< Velocity control mode
    TORQUE,        ///< Torque control mode
    CURRENT        ///< Current control mode
};

/**
 * @struct MotorState
 * @brief Structure containing current motor state
 */
struct MotorState {
    float position;      ///< Current position (radians or encoder counts)
    float velocity;      ///< Current velocity (rad/s or counts/s)
    float torque;        ///< Current torque (Nm)
    float current;       ///< Current current (A)
    MotorControlMode mode; ///< Current control mode
    bool enabled;        ///< Motor enable status
};

/**
 * @class Motors
 * @brief Main motor control class for the exoskeleton
 */
class Motors {
public:
    /**
     * @brief Constructor
     */
    Motors();

    /**
     * @brief Destructor
     */
    ~Motors();

    /**
     * @brief Initialize all motors
     * @return true if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * @brief Deinitialize all motors
     */
    void deinitialize();

    /**
     * @brief Enable a specific motor
     * @param motor_id Motor identifier (0 to MAX_MOTORS-1)
     * @return true if successful, false otherwise
     */
    bool enableMotor(uint8_t motor_id);

    /**
     * @brief Disable a specific motor
     * @param motor_id Motor identifier (0 to MAX_MOTORS-1)
     * @return true if successful, false otherwise
     */
    bool disableMotor(uint8_t motor_id);

    /**
     * @brief Set motor control mode
     * @param motor_id Motor identifier
     * @param mode Control mode to set
     * @return true if successful, false otherwise
     */
    bool setControlMode(uint8_t motor_id, MotorControlMode mode);

    /**
     * @brief Set position setpoint for a motor
     * @param motor_id Motor identifier
     * @param position Position setpoint (radians or encoder counts)
     * @return true if successful, false otherwise
     */
    bool setPosition(uint8_t motor_id, float position);

    /**
     * @brief Set velocity setpoint for a motor
     * @param motor_id Motor identifier
     * @param velocity Velocity setpoint (rad/s or counts/s)
     * @return true if successful, false otherwise
     */
    bool setVelocity(uint8_t motor_id, float velocity);

    /**
     * @brief Set torque setpoint for a motor
     * @param motor_id Motor identifier
     * @param torque Torque setpoint (Nm)
     * @return true if successful, false otherwise
     */
    bool setTorque(uint8_t motor_id, float torque);

    /**
     * @brief Get current motor state
     * @param motor_id Motor identifier
     * @param state Pointer to MotorState structure to fill
     * @return true if successful, false otherwise
     */
    bool getMotorState(uint8_t motor_id, MotorState* state);

    /**
     * @brief Update motor control loop (call periodically)
     * @return true if successful, false otherwise
     */
    bool update();

    /**
     * @brief Check if motors are initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;

private:
    bool initialized_;  ///< Initialization status flag

    // TODO: Add motor-specific data members
    // Example:
    // MotorState motor_states_[MAX_MOTORS];
    // MotorDriver* motor_drivers_[MAX_MOTORS];
    // uint8_t num_motors_;
};

#endif // MOTORS_H

