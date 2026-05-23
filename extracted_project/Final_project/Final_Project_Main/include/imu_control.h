#ifndef IMU_CONTROL_H
#define IMU_CONTROL_H

#include <Arduino.h>

struct OrientationData {
    float yaw, pitch, roll;     // Euler Angles (degrees)
    float accX, accY, accZ;     // Linear Acceleration (m/s^2)
    float gyroX, gyroY, gyroZ;  // Angular Velocity (rad/s)
    float qX, qY, qZ, qW;       // Quaternion for ROS EKF
    uint8_t sys, gyro, accel, mag; // Calibration Status (0-3)
};

extern OrientationData imu_data;
extern bool imu_ok;  // true = BNO055 init สำเร็จ, false = ข้ามการอ่าน I2C

void initIMU();
void updateIMU();
void displayCalibrationData();

#endif