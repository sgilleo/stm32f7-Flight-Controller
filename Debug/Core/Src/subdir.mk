################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AHRS.c \
../Core/Src/Battery.c \
../Core/Src/Flight_Modes.c \
../Core/Src/ICM42688P.c \
../Core/Src/Outputs.c \
../Core/Src/PIDs.c \
../Core/Src/Parameters.c \
../Core/Src/Quaternion.c \
../Core/Src/Sbus.c \
../Core/Src/USB.c \
../Core/Src/main.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c 

OBJS += \
./Core/Src/AHRS.o \
./Core/Src/Battery.o \
./Core/Src/Flight_Modes.o \
./Core/Src/ICM42688P.o \
./Core/Src/Outputs.o \
./Core/Src/PIDs.o \
./Core/Src/Parameters.o \
./Core/Src/Quaternion.o \
./Core/Src/Sbus.o \
./Core/Src/USB.o \
./Core/Src/main.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o 

C_DEPS += \
./Core/Src/AHRS.d \
./Core/Src/Battery.d \
./Core/Src/Flight_Modes.d \
./Core/Src/ICM42688P.d \
./Core/Src/Outputs.d \
./Core/Src/PIDs.d \
./Core/Src/Parameters.d \
./Core/Src/Quaternion.d \
./Core/Src/Sbus.d \
./Core/Src/USB.d \
./Core/Src/main.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AHRS.cyclo ./Core/Src/AHRS.d ./Core/Src/AHRS.o ./Core/Src/AHRS.su ./Core/Src/Battery.cyclo ./Core/Src/Battery.d ./Core/Src/Battery.o ./Core/Src/Battery.su ./Core/Src/Flight_Modes.cyclo ./Core/Src/Flight_Modes.d ./Core/Src/Flight_Modes.o ./Core/Src/Flight_Modes.su ./Core/Src/ICM42688P.cyclo ./Core/Src/ICM42688P.d ./Core/Src/ICM42688P.o ./Core/Src/ICM42688P.su ./Core/Src/Outputs.cyclo ./Core/Src/Outputs.d ./Core/Src/Outputs.o ./Core/Src/Outputs.su ./Core/Src/PIDs.cyclo ./Core/Src/PIDs.d ./Core/Src/PIDs.o ./Core/Src/PIDs.su ./Core/Src/Parameters.cyclo ./Core/Src/Parameters.d ./Core/Src/Parameters.o ./Core/Src/Parameters.su ./Core/Src/Quaternion.cyclo ./Core/Src/Quaternion.d ./Core/Src/Quaternion.o ./Core/Src/Quaternion.su ./Core/Src/Sbus.cyclo ./Core/Src/Sbus.d ./Core/Src/Sbus.o ./Core/Src/Sbus.su ./Core/Src/USB.cyclo ./Core/Src/USB.d ./Core/Src/USB.o ./Core/Src/USB.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su

.PHONY: clean-Core-2f-Src

