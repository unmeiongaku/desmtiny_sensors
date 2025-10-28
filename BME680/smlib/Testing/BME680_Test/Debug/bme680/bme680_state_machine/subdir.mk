################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bme680/bme680_state_machine/bme_active_object.c 

OBJS += \
./bme680/bme680_state_machine/bme_active_object.o 

C_DEPS += \
./bme680/bme680_state_machine/bme_active_object.d 


# Each subdirectory must supply rules for building sources it contributes
bme680/bme680_state_machine/%.o bme680/bme680_state_machine/%.su bme680/bme680_state_machine/%.cyclo: ../bme680/bme680_state_machine/%.c bme680/bme680_state_machine/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I"E:/Unmei Ongaku Workspace/Sensors/BME680/smlib/Testing/BME680_Test/timer" -I"E:/Unmei Ongaku Workspace/Sensors/BME680/smlib/Testing/BME680_Test/bme680" -I"E:/Unmei Ongaku Workspace/Sensors/BME680/smlib/Testing/BME680_Test/bme680/bme680_state_machine" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bme680-2f-bme680_state_machine

clean-bme680-2f-bme680_state_machine:
	-$(RM) ./bme680/bme680_state_machine/bme_active_object.cyclo ./bme680/bme680_state_machine/bme_active_object.d ./bme680/bme680_state_machine/bme_active_object.o ./bme680/bme680_state_machine/bme_active_object.su

.PHONY: clean-bme680-2f-bme680_state_machine

