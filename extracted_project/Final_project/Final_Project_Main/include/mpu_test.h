#ifndef MPU_TEST_H
#define MPU_TEST_H

#include <Arduino.h>

// เก็บค่า Raw Data จาก MPU6050
struct MPU_Raw_Data {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

// ใช้ชื่อ mpu_raw เพื่อไม่ให้ซ้ำกับชื่อ imu ในระบบ
extern MPU_Raw_Data mpu_raw;

void initMPU();
void readMPU();

#endif