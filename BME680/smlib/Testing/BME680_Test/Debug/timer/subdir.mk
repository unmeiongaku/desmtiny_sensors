################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../timer/timer.c 

OBJS += \
./timer/timer.o 

C_DEPS += \
./timer/timer.d 


# Each subdirectory must supply rules for building sources it contributes
timer/%.o timer/%.su timer/%.cyclo: ../timer/%.c timer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I"E:/Unmei Ongaku Workspace/Sensors/BME680/smlib/Testing/BME680_Test/timer" -I"E:/Unmei Ongaku Workspace/Sensors/BME680/smlib/Testing/BME680_Test/bme680" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-timer

clean-timer:
	-$(RM) ./timer/timer.cyclo ./timer/timer.d ./timer/timer.o ./timer/timer.su

.PHONY: clean-timer

