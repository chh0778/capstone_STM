################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/bsp.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/stm32f7xx_it.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/syscalls.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/system_clock.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/system_stm32f7xx.c 

C_DEPS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/bsp.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/stm32f7xx_it.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/syscalls.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/system_clock.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/system_stm32f7xx.d 

OBJS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/bsp.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/stm32f7xx_it.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/syscalls.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/system_clock.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/system_stm32f7xx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/%.o: ../Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/%.c Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/ROS" -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/turtlebot3_ros_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

