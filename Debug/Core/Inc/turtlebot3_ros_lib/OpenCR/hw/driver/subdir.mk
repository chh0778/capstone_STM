################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/delay.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/dma_stream_handlers.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_adc.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_can.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_dxl.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_eeprom.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_exti.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_i2c.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_micros.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_pwm.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_rtc.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_spi.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_timer.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_uart.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/flash.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/ring.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/vcp.c \
../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/wdg.c 

C_DEPS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/delay.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/dma_stream_handlers.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_adc.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_can.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_dxl.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_eeprom.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_exti.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_i2c.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_micros.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_pwm.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_rtc.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_spi.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_timer.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_uart.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/flash.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/ring.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/vcp.d \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/wdg.d 

OBJS += \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/delay.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/dma_stream_handlers.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_adc.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_can.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_dxl.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_eeprom.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_exti.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_i2c.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_micros.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_pwm.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_rtc.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_spi.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_timer.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/drv_uart.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/flash.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/ring.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/vcp.o \
./Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/wdg.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/%.o: ../Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/%.c Core/Inc/turtlebot3_ros_lib/OpenCR/hw/driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/ROS" -I"C:/Users/PC/STM32CubeIDE/workspace_1.7.0/LwIP_TCP_Client_201002_Cha/capstone_design3/Core/Inc/turtlebot3_ros_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

