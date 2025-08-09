################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/PID_Lib/PID_Lib.c 

OBJS += \
./Core/PID_Lib/PID_Lib.o 

C_DEPS += \
./Core/PID_Lib/PID_Lib.d 


# Each subdirectory must supply rules for building sources it contributes
Core/PID_Lib/%.o Core/PID_Lib/%.su Core/PID_Lib/%.cyclo: ../Core/PID_Lib/%.c Core/PID_Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-PID_Lib

clean-Core-2f-PID_Lib:
	-$(RM) ./Core/PID_Lib/PID_Lib.cyclo ./Core/PID_Lib/PID_Lib.d ./Core/PID_Lib/PID_Lib.o ./Core/PID_Lib/PID_Lib.su

.PHONY: clean-Core-2f-PID_Lib

