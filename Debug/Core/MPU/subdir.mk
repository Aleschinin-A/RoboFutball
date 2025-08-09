################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MPU/MPU.c 

OBJS += \
./Core/MPU/MPU.o 

C_DEPS += \
./Core/MPU/MPU.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MPU/%.o Core/MPU/%.su Core/MPU/%.cyclo: ../Core/MPU/%.c Core/MPU/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-MPU

clean-Core-2f-MPU:
	-$(RM) ./Core/MPU/MPU.cyclo ./Core/MPU/MPU.d ./Core/MPU/MPU.o ./Core/MPU/MPU.su

.PHONY: clean-Core-2f-MPU

