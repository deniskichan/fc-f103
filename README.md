## This is my STM32F103 (Blue Pill) simplified flight controller firmware project.

The goal is to build a simplified version of flight controller using:
- STM32CubeIDE
- HAL drivers
- FreeRTOS
- MPU6050 IMU

Right now the firmware reads IMU data, processes orientation (roll & pitch),
and prints values over UART for debugging.

What's implemented:
- I2C communication with MPU6050
- External interrupt from MPU6050
- FreeRTOS with multiple tasks
- Queue between sensor task and orientation task
- UART debug output (115200 baud)
- System clock configured to 72 MHz (HSE + PLL)

Tasks:
- SensorReadTask – reads raw IMU data when interrupt fires
- OrientationTask – computes roll & pitch
- DebugTask – prints orientation to UART

Hardware:
- STM32F103C8T6 (Blue Pill)
- MPU6050
- ST-Link V2

Connections:
- I2C SCL – PB6
- I2C SDA – PB7
- MPU6050 INT – PA11
- UART TX – PA9

How to build:
- Open the project in STM32CubeIDE
- Build
- Flash with ST-Link
- After flashing, open UART at 115200 to see roll/pitch values.

Why this project:

I'm building this to better understand:
- STM32 peripherals
- RTOS task scheduling
- Sensor communication over I2C
- Interrupt-driven data acquisition
- Basic orientation estimation

## Technical notes

### Filter tau choice

The current tau value was chosen as a balance between responsiveness and noise filtering.  
Lower tau makes the output more responsive but noisier; higher tau smooths noise but feels delayed.

### Known limitation near large angles

When pitch gets close to ±90°, roll estimation becomes unstable due to the sensor orientation and complementary algorithm.  
This may cause glitches in roll output near vertical orientations.
