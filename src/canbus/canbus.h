/**
 * @file canbus.h
 * @brief CAN bus communication class for exoskeleton embedded system
 * 
 * This class provides an interface for CAN bus communication, including
 * message sending, receiving, and filtering capabilities.
 */

#ifndef CANBUS_H
#define CANBUS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @struct CANMessage
 * @brief Structure representing a CAN message
 */
struct CANMessage {
    uint32_t id;        ///< CAN message ID (11-bit or 29-bit)
    uint8_t dlc;        ///< Data Length Code (0-8 bytes)
    uint8_t data[8];    ///< Message data payload
    bool extended;      ///< Extended frame format (29-bit ID) if true
    bool rtr;           ///< Remote Transmission Request if true
};

/**
 * @class CANBus
 * @brief Main CAN bus communication class for the exoskeleton
 */
class CANBus {
public:
    /**
     * @brief Constructor
     */
    CANBus();

    /**
     * @brief Destructor
     */
    ~CANBus();

    /**
     * @brief Initialize CAN bus interface
     * @param baudrate CAN bus baudrate (e.g., 125000, 250000, 500000, 1000000)
     * @return true if initialization successful, false otherwise
     */
    bool initialize(uint32_t baudrate);

    /**
     * @brief Deinitialize CAN bus interface
     */
    void deinitialize();

    /**
     * @brief Send a CAN message
     * @param message Pointer to CANMessage structure to send
     * @return true if message sent successfully, false otherwise
     */
    bool sendMessage(const CANMessage* message);

    /**
     * @brief Receive a CAN message (non-blocking)
     * @param message Pointer to CANMessage structure to fill
     * @return true if message received, false if no message available
     */
    bool receiveMessage(CANMessage* message);

    /**
     * @brief Set CAN message filter
     * @param filter_id Filter identifier
     * @param can_id CAN ID to filter for
     * @param mask CAN ID mask for filtering
     * @return true if filter set successfully, false otherwise
     */
    bool setFilter(uint8_t filter_id, uint32_t can_id, uint32_t mask);

    /**
     * @brief Clear CAN message filter
     * @param filter_id Filter identifier
     * @return true if filter cleared successfully, false otherwise
     */
    bool clearFilter(uint8_t filter_id);

    /**
     * @brief Get number of messages in receive buffer
     * @return Number of pending messages
     */
    uint32_t getPendingMessageCount() const;

    /**
     * @brief Check if CAN bus is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief Get error status
     * @return Error code (0 = no error)
     */
    uint32_t getErrorStatus() const;

private:
    bool initialized_;      ///< Initialization status flag
    uint32_t baudrate_;      ///< CAN bus baudrate
    uint32_t error_status_; ///< Error status register

    // TODO: Add CAN-specific data members
    // Example:
    // CANHandle* can_handle_;
    // CANMessage rx_buffer_[RX_BUFFER_SIZE];
    // uint32_t rx_buffer_head_;
    // uint32_t rx_buffer_tail_;
};

#endif // CANBUS_H

