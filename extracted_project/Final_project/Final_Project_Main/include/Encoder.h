#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// ฟังก์ชันสำหรับ setup
void setupEncoders();

// ฟังก์ชันสำหรับดึงค่าความเร็วและตำแหน่ง (Pass by Reference)
void getEncoderData(float &vL, float &vR, long &pL, long &pR, float dt);

#endif