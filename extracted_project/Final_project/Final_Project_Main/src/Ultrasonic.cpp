#include "Ultrasonic.h"

void setupUltrasonic(int tPin, int e1, int e2, int e3) {
    pinMode(tPin, OUTPUT);
    pinMode(e1, INPUT);
    pinMode(e2, INPUT);
    pinMode(e3, INPUT);
    digitalWrite(tPin, LOW);
}

float getDistance(int tPin, int ePin) {
    // 1. เคลียร์พิน Trig ให้ Low สนิท
    digitalWrite(tPin, LOW);
    delayMicroseconds(5);

    // 2. ยิง Pulse 20us (สำหรับ SR04M-2 กันน้ำ)
    digitalWrite(tPin, HIGH);
    delayMicroseconds(20); 
    digitalWrite(tPin, LOW);

    // 3. วัดช่วงเวลา Echo (Timeout 30ms)
    // STM32 จะนิ่งมากเพราะ Clock แรงกว่า ESP
    long duration = pulseIn(ePin, HIGH, 30000);

    // 4. วิเคราะห์ค่าที่ได้
    if (duration == 0) return -1.0; // No Signal (สายหลวมหรือไฟไม่พอ)
    
    float dist = (duration * 0.034) / 2.0;
    
    // กรองระยะบอด (20-25cm)
    if (dist < 20.0 || dist > 500.0) return -2.0; 
    
    return dist;
}