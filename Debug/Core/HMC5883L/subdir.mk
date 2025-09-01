################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HMC5883L/hmc5883l.c 

OBJS += \
./Core/HMC5883L/hmc5883l.o 

C_DEPS += \
./Core/HMC5883L/hmc5883l.d 


# Each subdirectory must supply rules for building sources it contributes
Core/HMC5883L/%.o Core/HMC5883L/%.su Core/HMC5883L/%.cyclo: ../Core/HMC5883L/%.c Core/HMC5883L/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-HMC5883L

clean-Core-2f-HMC5883L:
	-$(RM) ./Core/HMC5883L/hmc5883l.cyclo ./Core/HMC5883L/hmc5883l.d ./Core/HMC5883L/hmc5883l.o ./Core/HMC5883L/hmc5883l.su

.PHONY: clean-Core-2f-HMC5883L

