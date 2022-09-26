################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MyPrintf/MyPrintf.c 

OBJS += \
./Core/MyPrintf/MyPrintf.o 

C_DEPS += \
./Core/MyPrintf/MyPrintf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MyPrintf/%.o Core/MyPrintf/%.su: ../Core/MyPrintf/%.c Core/MyPrintf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Courses/CE437.N11 - ChuyenDeNhung1/CE437_Workspace/PracticeBoard/Core/CanProtocol" -I"D:/Courses/CE437.N11 - ChuyenDeNhung1/CE437_Workspace/PracticeBoard/Core/MyPrintf" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-MyPrintf

clean-Core-2f-MyPrintf:
	-$(RM) ./Core/MyPrintf/MyPrintf.d ./Core/MyPrintf/MyPrintf.o ./Core/MyPrintf/MyPrintf.su

.PHONY: clean-Core-2f-MyPrintf

