# Self-Balancing-Bot-
This repository contains the code and documentation for a Self-Balancing Robot developed using an Arduino. The robot is designed to maintain its balance using a combination of sensors, motors, and control algorithms.


Key Components:
Arduino: Acts as the main controller for processing sensor data and managing the motors.
Stepper Motors: Used for precise control of the wheels, allowing the bot to maintain balance.
Bluetooth Module: Enables wireless control and communication with the bot for calibration and commands.
Gyroscope/Accelerometer (IMU): Used to measure the orientation and tilt of the bot.
PID Control (Proportional, Integral, Derivative): A feedback loop mechanism used to stabilize the bot by adjusting motor speeds based on sensor data.
Features
Self-balancing: The bot uses data from a gyroscope/accelerometer to keep itself upright.
PID tuning: The bot's stability is improved through calibrated PID (Kp, Ki, Kd) values for finer control of motor speeds.
Bluetooth control: Remote control or tuning can be done via a Bluetooth-enabled device.
Hardware Required
Arduino (e.g., Uno, Mega)
2 x Stepper Motors with Drivers
MPU6050 Gyroscope/Accelerometer
Bluetooth Module (e.g., HC-05/HC-06)
Motor Driver Module (L298N or similar)
Power Source (e.g., Battery pack)
Frame/Chassis for the bot
Software
Programming Language: C/C++
IDE: Arduino IDE (or PlatformIO)
How It Works
The bot reads real-time orientation data from the gyroscope/accelerometer (MPU6050) to determine its tilt angle. Using the PID algorithm, it calculates the necessary adjustments to the stepper motors' speed and direction to maintain balance.

The Bluetooth module allows for wireless communication with the bot, making it possible to send commands or fine-tune the PID values on the go.

PID Tuning
Kp (Proportional): Adjusts based on the current tilt.
Ki (Integral): Reduces steady-state error by considering the cumulative error over time.
Kd (Derivative): Dampens the response based on the rate of change of the tilt.
The ideal PID values were fine-tuned through trial and error, and this repository contains the code used to adjust these values during testing.

Getting Started
Installation
Clone this repository:
bash
Copy code
git clone https://github.com/yourusername/self-balancing-bot.git
Open the project in the Arduino IDE.
Wiring
Stepper Motors connected to the motor drivers, controlled via the Arduino.
MPU6050 (gyroscope) wired to the Arduino I2C pins.
Bluetooth Module connected to the Arduino for wireless control.
Uploading Code
Open the self_balancing_bot.ino file in the Arduino IDE.
Adjust the Kp, Ki, and Kd values in the code to suit your hardware setup.
Upload the code to your Arduino.
Usage
Power On: When the bot is powered on, it will attempt to balance itself using the gyroscope data and PID control.
Bluetooth Control: Connect via a Bluetooth terminal app on your smartphone to send commands or adjust PID values wirelessly.
Troubleshooting
Bot not balancing: Ensure the MPU6050 is connected properly and reading data.
Motor jitter: Try recalibrating the PID values. Start with just adjusting Kp, then move to Ki and Kd.
Future Improvements
Adding obstacle detection and avoidance using ultrasonic sensors.
Implementing auto-calibration for PID values.
Exploring machine learning for better stability control.
License
This project is licensed under the MIT License - see the LICENSE file for details.

