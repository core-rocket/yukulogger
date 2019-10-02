################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32f303k6tx.s 

OBJS += \
./Startup/startup_stm32f303k6tx.o 


# Each subdirectory must supply rules for building sources it contributes
Startup/%.o: ../Startup/%.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I"D:/Downloads/SD_HAL_MPU6050-master" -I"D:/Downloads/BMP280_STM32-master/BMP280" -I"D:/Downloads/stm32-sdcard-master/sdcard" -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

