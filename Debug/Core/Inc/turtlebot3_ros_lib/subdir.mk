################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/turtlebot3_ros_lib/hw.c \
../Core/Inc/turtlebot3_ros_lib/wiring_pulse.c 

CPP_SRCS += \
../Core/Inc/turtlebot3_ros_lib/IMU.cpp \
../Core/Inc/turtlebot3_ros_lib/Tone.cpp \
../Core/Inc/turtlebot3_ros_lib/WMath.cpp \
../Core/Inc/turtlebot3_ros_lib/WString.cpp \
../Core/Inc/turtlebot3_ros_lib/duration.cpp \
../Core/Inc/turtlebot3_ros_lib/time.cpp \
../Core/Inc/turtlebot3_ros_lib/turtlebot3_diagnosis.cpp \
../Core/Inc/turtlebot3_ros_lib/turtlebot3_sensor.cpp 

C_DEPS += \
./Core/Inc/turtlebot3_ros_lib/hw.d \
./Core/Inc/turtlebot3_ros_lib/wiring_pulse.d 

OBJS += \
./Core/Inc/turtlebot3_ros_lib/IMU.o \
./Core/Inc/turtlebot3_ros_lib/Tone.o \
./Core/Inc/turtlebot3_ros_lib/WMath.o \
./Core/Inc/turtlebot3_ros_lib/WString.o \
./Core/Inc/turtlebot3_ros_lib/duration.o \
./Core/Inc/turtlebot3_ros_lib/hw.o \
./Core/Inc/turtlebot3_ros_lib/time.o \
./Core/Inc/turtlebot3_ros_lib/turtlebot3_diagnosis.o \
./Core/Inc/turtlebot3_ros_lib/turtlebot3_sensor.o \
./Core/Inc/turtlebot3_ros_lib/wiring_pulse.o 

CPP_DEPS += \
./Core/Inc/turtlebot3_ros_lib/IMU.d \
./Core/Inc/turtlebot3_ros_lib/Tone.d \
./Core/Inc/turtlebot3_ros_lib/WMath.d \
./Core/Inc/turtlebot3_ros_lib/WString.d \
./Core/Inc/turtlebot3_ros_lib/duration.d \
./Core/Inc/turtlebot3_ros_lib/time.d \
./Core/Inc/turtlebot3_ros_lib/turtlebot3_diagnosis.d \
./Core/Inc/turtlebot3_ros_lib/turtlebot3_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/turtlebot3_ros_lib/%.o: ../Core/Inc/turtlebot3_ros_lib/%.cpp Core/Inc/turtlebot3_ros_lib/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/turtlebot3_ros_lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Inc/turtlebot3_ros_lib/%.o: ../Core/Inc/turtlebot3_ros_lib/%.c Core/Inc/turtlebot3_ros_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/turtlebot3_ros_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

