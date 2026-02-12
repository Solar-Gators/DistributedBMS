################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/BmsCanInterface.cpp \
../Core/Src/BmsCanProtocol.cpp \
../Core/Src/BmsManager.cpp \
../Core/Src/CanFdBus.cpp \
../Core/Src/PrimaryBmsFleet.cpp \
../Core/Src/User.cpp \
../Core/Src/ads1115.cpp \
../Core/Src/bts71040.cpp \
../Core/Src/ina226.cpp \
../Core/Src/lsm6dso32.cpp \
../Core/Src/main.cpp 

C_SRCS += \
../Core/Src/app_freertos.c \
../Core/Src/stm32l5xx_hal_msp.c \
../Core/Src/stm32l5xx_hal_timebase_tim.c \
../Core/Src/stm32l5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l5xx.c 

C_DEPS += \
./Core/Src/app_freertos.d \
./Core/Src/stm32l5xx_hal_msp.d \
./Core/Src/stm32l5xx_hal_timebase_tim.d \
./Core/Src/stm32l5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l5xx.d 

OBJS += \
./Core/Src/BmsCanInterface.o \
./Core/Src/BmsCanProtocol.o \
./Core/Src/BmsManager.o \
./Core/Src/CanFdBus.o \
./Core/Src/PrimaryBmsFleet.o \
./Core/Src/User.o \
./Core/Src/ads1115.o \
./Core/Src/app_freertos.o \
./Core/Src/bts71040.o \
./Core/Src/ina226.o \
./Core/Src/lsm6dso32.o \
./Core/Src/main.o \
./Core/Src/stm32l5xx_hal_msp.o \
./Core/Src/stm32l5xx_hal_timebase_tim.o \
./Core/Src/stm32l5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l5xx.o 

CPP_DEPS += \
./Core/Src/BmsCanInterface.d \
./Core/Src/BmsCanProtocol.d \
./Core/Src/BmsManager.d \
./Core/Src/CanFdBus.d \
./Core/Src/PrimaryBmsFleet.d \
./Core/Src/User.d \
./Core/Src/ads1115.d \
./Core/Src/bts71040.d \
./Core/Src/ina226.d \
./Core/Src/lsm6dso32.d \
./Core/Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/RTOS2/Include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/RTOS2/Include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BmsCanInterface.cyclo ./Core/Src/BmsCanInterface.d ./Core/Src/BmsCanInterface.o ./Core/Src/BmsCanInterface.su ./Core/Src/BmsCanProtocol.cyclo ./Core/Src/BmsCanProtocol.d ./Core/Src/BmsCanProtocol.o ./Core/Src/BmsCanProtocol.su ./Core/Src/BmsManager.cyclo ./Core/Src/BmsManager.d ./Core/Src/BmsManager.o ./Core/Src/BmsManager.su ./Core/Src/CanFdBus.cyclo ./Core/Src/CanFdBus.d ./Core/Src/CanFdBus.o ./Core/Src/CanFdBus.su ./Core/Src/PrimaryBmsFleet.cyclo ./Core/Src/PrimaryBmsFleet.d ./Core/Src/PrimaryBmsFleet.o ./Core/Src/PrimaryBmsFleet.su ./Core/Src/User.cyclo ./Core/Src/User.d ./Core/Src/User.o ./Core/Src/User.su ./Core/Src/ads1115.cyclo ./Core/Src/ads1115.d ./Core/Src/ads1115.o ./Core/Src/ads1115.su ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/bts71040.cyclo ./Core/Src/bts71040.d ./Core/Src/bts71040.o ./Core/Src/bts71040.su ./Core/Src/ina226.cyclo ./Core/Src/ina226.d ./Core/Src/ina226.o ./Core/Src/ina226.su ./Core/Src/lsm6dso32.cyclo ./Core/Src/lsm6dso32.d ./Core/Src/lsm6dso32.o ./Core/Src/lsm6dso32.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l5xx_hal_msp.cyclo ./Core/Src/stm32l5xx_hal_msp.d ./Core/Src/stm32l5xx_hal_msp.o ./Core/Src/stm32l5xx_hal_msp.su ./Core/Src/stm32l5xx_hal_timebase_tim.cyclo ./Core/Src/stm32l5xx_hal_timebase_tim.d ./Core/Src/stm32l5xx_hal_timebase_tim.o ./Core/Src/stm32l5xx_hal_timebase_tim.su ./Core/Src/stm32l5xx_it.cyclo ./Core/Src/stm32l5xx_it.d ./Core/Src/stm32l5xx_it.o ./Core/Src/stm32l5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l5xx.cyclo ./Core/Src/system_stm32l5xx.d ./Core/Src/system_stm32l5xx.o ./Core/Src/system_stm32l5xx.su

.PHONY: clean-Core-2f-Src

