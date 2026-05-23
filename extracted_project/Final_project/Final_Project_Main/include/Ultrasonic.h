#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// ฟังก์ชันสำหรับตั้งค่าพิน
void setupUltrasonic(int tPin, int e1, int e2, int e3);

// ฟังก์ชันสำหรับอ่านระยะทางรายตัว (Return ค่า -1 คือ Error, -2 คือ Out of Range)
float getDistance(int tPin, int ePin);

#endif