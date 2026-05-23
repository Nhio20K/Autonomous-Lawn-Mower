#include "mpu_test.h"
#include <Wire.h>

const uint8_t MPU_ADDR = 0x68; 
MPU_Raw_Data mpu_raw;

void initMPU() {
    Wire.begin(); 
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // register พลังงาน
    Wire.write(0);    // ปลุก MPU6050
    if (Wire.endTransmission() != 0) {
        Serial1.println("MPU6050 connection failed!");
    }
}

void readMPU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    
    // Casting ค่าให้ตรงประเภทเพื่อแก้ปัญหา Ambiguous ที่คุณเจอ
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);

    if (Wire.available() >= 14) {
        mpu_raw.ax = Wire.read() << 8 | Wire.read();
        mpu_raw.ay = Wire.read() << 8 | Wire.read();
        mpu_raw.az = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // ข้าม Temperature
        mpu_raw.gx = Wire.read() << 8 | Wire.read();
        mpu_raw.gy = Wire.read() << 8 | Wire.read();
        mpu_raw.gz = Wire.read() << 8 | Wire.read();
    }
}