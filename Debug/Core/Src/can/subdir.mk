################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/can/can_app.cpp \
../Core/Src/can/can_bus_stm32.cpp 

OBJS += \
./Core/Src/can/can_app.o \
./Core/Src/can/can_bus_stm32.o 

CPP_DEPS += \
./Core/Src/can/can_app.d \
./Core/Src/can/can_bus_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/can/%.o Core/Src/can/%.su Core/Src/can/%.cyclo: ../Core/Src/can/%.cpp Core/Src/can/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-can

clean-Core-2f-Src-2f-can:
	-$(RM) ./Core/Src/can/can_app.cyclo ./Core/Src/can/can_app.d ./Core/Src/can/can_app.o ./Core/Src/can/can_app.su ./Core/Src/can/can_bus_stm32.cyclo ./Core/Src/can/can_bus_stm32.d ./Core/Src/can/can_bus_stm32.o ./Core/Src/can/can_bus_stm32.su

.PHONY: clean-Core-2f-Src-2f-can

