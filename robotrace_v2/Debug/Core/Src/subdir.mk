################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMI088.c \
../Core/Src/PIDcontrol.c \
../Core/Src/SDcard.c \
../Core/Src/WS2812C.c \
../Core/Src/battery.c \
../Core/Src/control.c \
../Core/Src/encoder.c \
../Core/Src/fatfs_sd.c \
../Core/Src/lineSensor.c \
../Core/Src/main.c \
../Core/Src/markerSensor.c \
../Core/Src/motor.c \
../Core/Src/setup.c \
../Core/Src/ssd1306.c \
../Core/Src/ssd1306_fonts.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/switch.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/timer.c 

OBJS += \
./Core/Src/BMI088.o \
./Core/Src/PIDcontrol.o \
./Core/Src/SDcard.o \
./Core/Src/WS2812C.o \
./Core/Src/battery.o \
./Core/Src/control.o \
./Core/Src/encoder.o \
./Core/Src/fatfs_sd.o \
./Core/Src/lineSensor.o \
./Core/Src/main.o \
./Core/Src/markerSensor.o \
./Core/Src/motor.o \
./Core/Src/setup.o \
./Core/Src/ssd1306.o \
./Core/Src/ssd1306_fonts.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/switch.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/timer.o 

C_DEPS += \
./Core/Src/BMI088.d \
./Core/Src/PIDcontrol.d \
./Core/Src/SDcard.d \
./Core/Src/WS2812C.d \
./Core/Src/battery.d \
./Core/Src/control.d \
./Core/Src/encoder.d \
./Core/Src/fatfs_sd.d \
./Core/Src/lineSensor.d \
./Core/Src/main.d \
./Core/Src/markerSensor.d \
./Core/Src/motor.d \
./Core/Src/setup.d \
./Core/Src/ssd1306.d \
./Core/Src/ssd1306_fonts.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/switch.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMI088.cyclo ./Core/Src/BMI088.d ./Core/Src/BMI088.o ./Core/Src/BMI088.su ./Core/Src/PIDcontrol.cyclo ./Core/Src/PIDcontrol.d ./Core/Src/PIDcontrol.o ./Core/Src/PIDcontrol.su ./Core/Src/SDcard.cyclo ./Core/Src/SDcard.d ./Core/Src/SDcard.o ./Core/Src/SDcard.su ./Core/Src/WS2812C.cyclo ./Core/Src/WS2812C.d ./Core/Src/WS2812C.o ./Core/Src/WS2812C.su ./Core/Src/battery.cyclo ./Core/Src/battery.d ./Core/Src/battery.o ./Core/Src/battery.su ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/fatfs_sd.cyclo ./Core/Src/fatfs_sd.d ./Core/Src/fatfs_sd.o ./Core/Src/fatfs_sd.su ./Core/Src/lineSensor.cyclo ./Core/Src/lineSensor.d ./Core/Src/lineSensor.o ./Core/Src/lineSensor.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/markerSensor.cyclo ./Core/Src/markerSensor.d ./Core/Src/markerSensor.o ./Core/Src/markerSensor.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/setup.cyclo ./Core/Src/setup.d ./Core/Src/setup.o ./Core/Src/setup.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/ssd1306_fonts.cyclo ./Core/Src/ssd1306_fonts.d ./Core/Src/ssd1306_fonts.o ./Core/Src/ssd1306_fonts.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/switch.cyclo ./Core/Src/switch.d ./Core/Src/switch.o ./Core/Src/switch.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/timer.cyclo ./Core/Src/timer.d ./Core/Src/timer.o ./Core/Src/timer.su

.PHONY: clean-Core-2f-Src

