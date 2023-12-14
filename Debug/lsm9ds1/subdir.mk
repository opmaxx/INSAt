################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lsm9ds1/lsm9ds1_regs.c 

OBJS += \
./lsm9ds1/lsm9ds1_regs.o 

C_DEPS += \
./lsm9ds1/lsm9ds1_regs.d 


# Each subdirectory must supply rules for building sources it contributes
lsm9ds1/lsm9ds1_regs.o: ../lsm9ds1/lsm9ds1_regs.c lsm9ds1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../lsm9ds1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lsm9ds1

clean-lsm9ds1:
	-$(RM) ./lsm9ds1/lsm9ds1_regs.cyclo ./lsm9ds1/lsm9ds1_regs.d ./lsm9ds1/lsm9ds1_regs.o ./lsm9ds1/lsm9ds1_regs.su

.PHONY: clean-lsm9ds1

