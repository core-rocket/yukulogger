################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Downloads/SD_HAL_MPU6050-master/sd_hal_mpu6050.c 

OBJS += \
./SD_HAL_MPU6050-master/sd_hal_mpu6050.o 

C_DEPS += \
./SD_HAL_MPU6050-master/sd_hal_mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
SD_HAL_MPU6050-master/sd_hal_mpu6050.o: D:/Downloads/SD_HAL_MPU6050-master/sd_hal_mpu6050.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I"D:/Downloads/BMP280_STM32-master/BMP280" -I"D:/Downloads/SD_HAL_MPU6050-master" -I"D:/Downloads/stm32-sdcard-master/sdcard" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"SD_HAL_MPU6050-master/sd_hal_mpu6050.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

