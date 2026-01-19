/**
  ******************************************************************************
  * @file           : can_frame.hpp
  * @brief          : Header file for declaring the structure and initial state
  * 				  of a can frame (ID, DLC, and up to 8 data bytes).
  ******************************************************************************
*/
#pragma once

#include <cstdint>

struct CanFrame {
  uint32_t id = 0; // Base ID is the first 11-bit MSB
  uint8_t  dlc = 0; // Number of bytes of data
  uint8_t  data[8] = {0}; // Data to be transmitted
};
