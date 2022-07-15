################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/startup/startup_stm32f746xx.S 

OBJS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/startup/startup_stm32f746xx.o 

S_UPPER_DEPS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/startup/startup_stm32f746xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/startup/%.o: ../Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/startup/%.S Core/Inc/turtlebot3_ros_lib/OpenCR/bsp/opencr/startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

