/**
  ******************************************************************************
  * @file           : can_app.h
  * @brief          : C/C++ boundary for the CAN application layer.
  *
  * This header declares a small C-callable API implemented in can_app.cpp.
  * It allows CubeMX-generated C code (e.g., main.c / HAL callbacks) to call
  * into the C++ CAN module by using extern "C" when included from C++.
  ******************************************************************************
*/
#pragma once

#ifdef __cplusplus
extern "C"{
#endif

void CanApp_Init(void);
void CanApp_Tick(void);

#ifdef __cplusplus
}
#endif
