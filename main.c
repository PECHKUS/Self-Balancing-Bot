// Include necessary libraries
#include "I2Cdev.h"                              // Part of the I2Cdev library (https://github.com/jrowberg/i2cdevlib)
#include <PID_v1.h>                              // PID library (https://github.com/br3ttb/Arduino-PID-Library)
#include "MPU6050_6Axis_MotionApps20.h"         // MPU6050 library (https://github.com/jrowberg/i2cdevlib)
// Include Wire library for I2C communication
#include <Wire.h>                                // Built-in Arduino library for I2C communication

// Define constants and pin assignments
#define motorPin1 6
#define motorPin2 9
#define motorPin3 10
#define motorPin4 11

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // Set to true if DMP init is successful
uint8_t mpuIntStatus;   // Interrupt status from MPU
uint8_t devStatus;      // Return status after each device operation
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];           // [yaw, pitch, roll]

// PID controller variables
double setpoint = 176; // Set when bot is upright
double Kp = 21, Ki = 140, Kd = 0.8;
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// MPU interrupt flag
volatile bool mpuInterrupt = false;

// Interrupt detection routine
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);

    // MPU6050 setup
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Load and configure DMP
    devStatus = mpu.dmpInitialize();

    // Set gyro offsets
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        // Setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);

    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // Initialize motor control pins
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);

    // Stop the motors by default
    analogWrite(motorPin1, LOW);
    analogWrite(motorPin2, LOW);
    analogWrite(motorPin3, LOW);
    analogWrite(motorPin4, LOW);
}

void loop() {
    // Do nothing if DMP initialization failed
    if (!dmpReady) return;

    // Wait for MPU interrupt or new DMP packet
    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();

        // Debug output for tuning via serial monitor
        Serial.print(input); Serial.print(" => "); Serial.println(output);

        if (input > 150 && input < 200) {
            if (output > 0) Forward();
            else if (output < 0) Reverse();
        } else {
            Stop();
        }
    }

    // Reset interrupt flag
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    // Check for FIFO overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // Get Quaternion and YPR (Yaw, Pitch, Roll) values
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        input = ypr[1] * 180 / M_PI + 180; // Using pitch for balancing
    }
}

// Forward motor function
void Forward() {
    analogWrite(motorPin1, output);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, output);
    analogWrite(motorPin4, 0);
    Serial.print("F");
}

// Reverse motor function
void Reverse() {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, output * -1);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, output * -1);
    Serial.print("R");
}

// Stop motor function
void Stop() {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 0);
    Serial.print("S");
}
