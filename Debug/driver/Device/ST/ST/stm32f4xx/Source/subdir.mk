################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Device/ST/ST/stm32f4xx/Source/system_stm32f4xx.c 

OBJS += \
./driver/Device/ST/ST/stm32f4xx/Source/system_stm32f4xx.o 

C_DEPS += \
./driver/Device/ST/ST/stm32f4xx/Source/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Device/ST/ST/stm32f4xx/Source/%.o driver/Device/ST/ST/stm32f4xx/Source/%.su driver/Device/ST/ST/stm32f4xx/Source/%.cyclo: ../driver/Device/ST/ST/stm32f4xx/Source/%.c driver/Device/ST/ST/stm32f4xx/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"E:/Github/git_repository/FreeRTOS_Kernal/RTOS_Schedular/Round_Robin_Scheduler/driver/CMSIS/Core/Include" -I"E:/Github/git_repository/FreeRTOS_Kernal/RTOS_Schedular/Round_Robin_Scheduler/driver/Device/ST/ST/stm32f4xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-driver-2f-Device-2f-ST-2f-ST-2f-stm32f4xx-2f-Source

clean-driver-2f-Device-2f-ST-2f-ST-2f-stm32f4xx-2f-Source:
	-$(RM) ./driver/Device/ST/ST/stm32f4xx/Source/system_stm32f4xx.cyclo ./driver/Device/ST/ST/stm32f4xx/Source/system_stm32f4xx.d ./driver/Device/ST/ST/stm32f4xx/Source/system_stm32f4xx.o ./driver/Device/ST/ST/stm32f4xx/Source/system_stm32f4xx.su

.PHONY: clean-driver-2f-Device-2f-ST-2f-ST-2f-stm32f4xx-2f-Source

