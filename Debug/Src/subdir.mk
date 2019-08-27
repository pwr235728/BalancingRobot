################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/mpu6050.c \
../Src/quaternion.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/vector3.c 

OBJS += \
./Src/main.o \
./Src/mpu6050.o \
./Src/quaternion.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/vector3.o 

C_DEPS += \
./Src/main.d \
./Src/mpu6050.d \
./Src/quaternion.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/vector3.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DUSE_FULL_LL_DRIVER '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DSTM32F103xB -DUSE_HAL_DRIVER -I"C:/Users/Kurat/Documents/git_repos/balancing_f1/Inc" -I"C:/Users/Kurat/Documents/git_repos/balancing_f1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Kurat/Documents/git_repos/balancing_f1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Kurat/Documents/git_repos/balancing_f1/Drivers/CMSIS/Include" -I"C:/Users/Kurat/Documents/git_repos/balancing_f1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


