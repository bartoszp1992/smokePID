################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Display/an_disp.c 

OBJS += \
./Core/Display/an_disp.o 

C_DEPS += \
./Core/Display/an_disp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Display/%.o: ../Core/Display/%.c Core/Display/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L041xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Display

clean-Core-2f-Display:
	-$(RM) ./Core/Display/an_disp.d ./Core/Display/an_disp.o

.PHONY: clean-Core-2f-Display

