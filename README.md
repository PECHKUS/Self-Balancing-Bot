**🛠️ Self-Balancing Robot Using Arduino**

This project demonstrates a self-balancing robot using an Arduino Uno, controlled by a PID algorithm. The robot maintains balance by detecting its tilt angle with an MPU6050 Gyroscope/Accelerometer and adjusts motor speeds accordingly using a L298N Motor Driver.

**🚩 Project Objective**

To build a robot that autonomously maintains balance on two wheels by continuously adjusting motor movements based on real-time tilt data provided by a gyroscope sensor.

**⚙️ Hardware Components**

Arduino Uno: Microcontroller that processes sensor data and controls the motors.
MPU6050: 6-axis gyroscope and accelerometer for detecting the robot’s tilt (yaw, pitch, roll).
2 x Geared DC Motors: Drives the robot's wheels forward and backward.
L298N Motor Driver: H-bridge motor driver for controlling motor direction and speed.
7.4V Li-ion Battery: Powers the Arduino and the motors.
Wheels & Chassis: Robot base structure.
Wiring and Connectors: Basic connectors and wiring to integrate the components.


**🧠 Control System Overview**

MPU6050 Sensor
The MPU6050 detects the robot's tilt. It continuously feeds the Yaw, Pitch, and Roll (YPR) data to the Arduino. The pitch angle is primarily used to determine the robot's inclination.

PID Controller
A PID (Proportional, Integral, Derivative) control system is implemented to keep the robot balanced:

Kp (Proportional): Reacts to the current tilt angle.

Ki (Integral): Corrects accumulated past errors.

Kd (Derivative): Predicts future tilt behavior to stabilize movements.

Tuning Values:

Setpoint: 176 (the angle when the robot is perfectly upright).

Kp: 21 (handles immediate correction).

Ki: 140 (compensates for persistent drift).

Kd: 0.8 (smoothens response).

The PID controller adjusts the PWM values fed to the motors, controlling their speed and direction to maintain the robot's balance.


**🔑 Key Functions in Code**


MPU6050 Initialization Initializes the I2C connection with the MPU6050, calibrates the sensor, and sets the gyro offsets.


mpu.initialize();

mpu.setXGyroOffset(220);

mpu.setYGyroOffset(76);

mpu.setZGyroOffset(-85);

mpu.setZAccelOffset(1688);

PID Calculation The PID library computes motor adjustments based on real-time input (tilt) from the sensor.



pid.Compute();

Motor Control The robot moves forward or backward based on the PID output. If the robot tilts forward, the wheels spin forward, and if it tilts backward, the wheels reverse.


if (output > 0) Forward();

else Reverse();

Reading Tilt Data The DMP (Digital Motion Processor) of the MPU6050 processes sensor data for faster readings. The pitch value is used as the input for the PID controller.


mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

input = ypr[1] * 180/M_PI + 180;


**
**🧩 Code Structure****

robot_code.ino: The main Arduino code file that contains:

MPU6050 setup and interrupt handling.

PID tuning and motor control logic.

Functions to drive the motors: Forward(), Reverse(), and Stop().

Main Loops & Logic

The loop() function continuously monitors the MPU6050 data and performs PID computations to adjust the motor speed.

Interrupts are used to efficiently read sensor data from the MPU6050.

**💡 Conclusion**

Building a self-balancing robot is a fantastic project that introduces you to the world of sensors, feedback control, and motor control. We hope you find this guide helpful as you build your own version of the bot. Don’t forget to experiment, learn, and have fun!

Stay tuned for more updates as we continue to improve the project and explore new features
