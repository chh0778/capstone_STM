################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/ROS/duration.cpp \
../Core/Inc/ROS/time.cpp 

OBJS += \
./Core/Inc/ROS/duration.o \
./Core/Inc/ROS/time.o 

CPP_DEPS += \
./Core/Inc/ROS/duration.d \
./Core/Inc/ROS/time.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/ROS/%.o: ../Core/Inc/ROS/%.cpp Core/Inc/ROS/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/ROS" -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/turtlebot3_ros_lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

