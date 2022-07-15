################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/turtlebot3_ros_lib/OpenCR/main.cpp \
../Core/Inc/turtlebot3_ros_lib/OpenCR/variant.cpp 

OBJS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/main.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/variant.o 

CPP_DEPS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/main.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/variant.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/turtlebot3_ros_lib/OpenCR/%.o: ../Core/Inc/turtlebot3_ros_lib/OpenCR/%.cpp Core/Inc/turtlebot3_ros_lib/OpenCR/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/ROS" -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/turtlebot3_ros_lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

