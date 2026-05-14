################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/as5600.c \
../Src/main.c \
../Src/stepper.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/as5600.o \
./Src/main.o \
./Src/stepper.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/as5600.d \
./Src/main.d \
./Src/stepper.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F303K8Tx -DNUCLEO_F303K8 -DSTM32F3 -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/as5600.cyclo ./Src/as5600.d ./Src/as5600.o ./Src/as5600.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/stepper.cyclo ./Src/stepper.d ./Src/stepper.o ./Src/stepper.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

