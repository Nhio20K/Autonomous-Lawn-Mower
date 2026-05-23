#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Servo.h>

// กำหนดพินใหม่สำหรับระบบตีนตะขาบ 2 มอเตอร์
#define MOTOR_L_PIN PB4  // คุมสายพานซ้าย
#define MOTOR_R_PIN PB5  // คุมสายพานขวา

#define MAX_SPEED_MS    1.25f  // วัดจริงแล้ว (m/s)
#define DEADBAND_MS     0.05f  // ⚠️ ปรับได้: ต่ำกว่านี้ → หยุดเลย (m/s)
#define MIN_PWM_OFFSET  60     // ⚠️ ปรับได้: µs ต่ำสุดจาก 1500 ที่มอเตอร์จะขยับได้จริง

void initMotors();
void driveMotors(float vL_ms, float vR_ms); // รับค่าเป็น m/s
void stopMotors();
void stop();

#endif