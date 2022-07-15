################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/hw.c 

C_DEPS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/hw.d 

OBJS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/hw.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/turtlebot3_ros_lib/OpenCR/hw/%.o: ../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/%.c Core/Inc/turtlebot3_ros_lib/OpenCR/hw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/ROS" -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/turtlebot3_ros_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

