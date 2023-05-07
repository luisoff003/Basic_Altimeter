################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sensors/Src/BMX055.c \
../Sensors/Src/bmp280.c 

OBJS += \
./Sensors/Src/BMX055.o \
./Sensors/Src/bmp280.o 

C_DEPS += \
./Sensors/Src/BMX055.d \
./Sensors/Src/bmp280.d 


# Each subdirectory must supply rules for building sources it contributes
Sensors/Src/%.o Sensors/Src/%.su: ../Sensors/Src/%.c Sensors/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/luis0/STM32CubeIDE/workspace_1.9.0/Basic_Altimeter/Sensors/Inc" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Sensors-2f-Src

clean-Sensors-2f-Src:
	-$(RM) ./Sensors/Src/BMX055.d ./Sensors/Src/BMX055.o ./Sensors/Src/BMX055.su ./Sensors/Src/bmp280.d ./Sensors/Src/bmp280.o ./Sensors/Src/bmp280.su

.PHONY: clean-Sensors-2f-Src

