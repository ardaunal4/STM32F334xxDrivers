################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/toggle_led.c \
../Src/toggle_led_with_button.c 

OBJS += \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/toggle_led.o \
./Src/toggle_led_with_button.o 

C_DEPS += \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/toggle_led.d \
./Src/toggle_led_with_button.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F334C8Tx -DSTM32 -DSTM32F3348_DISCO -DSTM32F3 -c -I../Inc -I"D:/EmbeddedC/lecture2/stm32f334xx_drivers/drivers/Src" -I"D:/EmbeddedC/lecture2/stm32f334xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/toggle_led.cyclo ./Src/toggle_led.d ./Src/toggle_led.o ./Src/toggle_led.su ./Src/toggle_led_with_button.cyclo ./Src/toggle_led_with_button.d ./Src/toggle_led_with_button.o ./Src/toggle_led_with_button.su

.PHONY: clean-Src

