# Self-Balancing Robot

A two-wheeled robot that balances itself using PID control and an IMU sensor.
![WhatsApp Image 2025-05-06 at 19 54 33_92583673](https://github.com/user-attachments/assets/fb2dc30d-2523-4773-9ac1-20db582010bb)

## Overview

This project implements a self-balancing robot using an LPC1768 microcontroller that reads values from a Sparkfun 9DOF IMU and uses a PID control system to maintain balance. The robot uses an L298N motor driver to drive two DAGU DG01D motors powered by two 6V batteries connected in series.

## Components

- LPC1768 microcontroller
- Sparkfun 9DOF IMU sensor
- L298N motor driver
- 2 x DAGU DG01D motors
- 2 x 6V batteries (connected in series to provide 12V)
- 3D printed robot structure

## How It Works

### IMU (Inertial Measurement Unit)

The 9DOF IMU provides accelerometer and gyroscope readings in 3 axes (x, y, z). The system:
- Wakes up the IMU from sleep mode by writing to register 1
- Uses I2C communication protocol
- Has accelerometer and gyroscope scales of ±2g and ±250 deg/s respectively
- Uses two 10kΩ pull-up resistors on SDA and SSL inputs
- Calculates the tilt angle using the accelerometer readings

### Motor Control

The L298N motor driver:
- Controls two DC motors simultaneously
- Supplies approximately 8.5V to each motor (after 3.5V voltage drop)
- Uses PWM frequency of 25 kHz
- Controls motor states (FORWARD, BACKWARD, BRAKE, STOP) via control pins

### PID Controller

The Proportional-Integral-Derivative (PID) controller:
- Takes the tilt error as input and outputs motor control signals
- Runs at 300 Hz frequency (updates every 1/300 seconds)
- Uses tunable parameters Kp, Ki, and Kd for stability
- Includes a 5° tolerance zone for the STOP state
- Uses gyroscope readings directly for derivative calculations

## 3D Printing Design
![image_2025-05-06_14-32-43](https://github.com/user-attachments/assets/f44834ba-e801-4239-b01c-0d15b295514d)

The 3D design files for the robot structure can be found at: [3D Print Files](https://makerworld.com/en/models/691734-fall-e-the-self-balancing-robot?from=search#profileId-620442)

*Note: The current design is quite heavy, which affects the robot's ability to balance against gravity's torque at higher angular accelerations. Consider using lighter materials or a redesigned structure for better performance.*

## Code

The full project code is available on GitHub:
[https://github.com/Sameer-13/Self-Balancing-Robot](https://github.com/Sameer-13/Self-Balancing-Robot)

## Challenges Faced

1. Rapid PWM value changes during tuning caused overheating issues, burning two motor drivers
2. The initial 3D-printed structure was too heavy, limiting the robot's balancing capability
3. Replacing the burned motor driver required redesigning the robot to accommodate a larger driver and additional batteries

## Key Parameters

The project uses these key parameters for best performance:

- PID frequency: 300 Hz
- PWM frequency: 25 kHz
- Kp: 7.5
- Ki: 0.5
- Kd: 1.0
- ANGLE_TOLERANCE: 5° (±5° range for STOP state)

## Development Environment

The project was developed using:
- mbed library/RTOS
- Keil Cloud Studios (studio.keil.arm.com)
- API documentation: https://os.mbed.com/docs/mbed-os/v6.16/apis/index.html
