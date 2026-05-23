#include "Encoder.h"
#include <Arduino.h>

// กำหนดพิน (PA0, PA1 ล้อซ้าย | PA4, PA5 ล้อขวา)
const int pinAL = PA0; const int pinBL = PA1;
const int pinAR = PA4; const int pinBR = PA5;

// ตัวแปรเก็บค่า Pulse (volatile สำหรับ Interrupt)
volatile long pulseL = 0;
volatile long pulseR = 0;

// ตัวแปรช่วยเช็คสถานะเพื่อป้องกันการนับทิศทางผิด (State Tracking)
volatile bool lastAL = false;
volatile bool lastBL = false;
volatile bool lastAR = false;
volatile bool lastBR = false;

// ตัวแปรสำหรับคำนวณความเร็ว
static long prevPulseL = 0;
static long prevPulseR = 0;
static float filtered_vL = 0;
static float filtered_vR = 0;

// ค่า Alpha สำหรับ Low-Pass Filter (0.0 - 1.0)
// ยิ่งน้อยยิ่งนิ่ง (ช้า) | ยิ่งมากยิ่งตอบสนองเร็ว (กระโดด)
const float alpha = 0.5;

// --- Interrupt Service Routines (ISRs) แบบ Fast Register Read ---
// ใช้ GPIOA->IDR อ่านทั้ง port พร้อมกันใน snapshot เดียว
// เร็วกว่า digitalRead() ประมาณ 10x และป้องกัน glitch ระหว่าง A กับ B

void handleEncoderL() {
    // อ่าน GPIOA register ทั้ง port ในครั้งเดียว (atomic snapshot)
    uint32_t idr = GPIOA->IDR;
    bool A = (idr & (1U << 0)) != 0;  // PA0
    bool B = (idr & (1U << 1)) != 0;  // PA1

    if (A != lastAL) {
        if (A == B) pulseL++; else pulseL--;
    } else if (B != lastBL) {
        if (A == B) pulseL--; else pulseL++;
    }

    lastAL = A;
    lastBL = B;
}

void handleEncoderR() {
    // อ่าน GPIOA register ทั้ง port ในครั้งเดียว (atomic snapshot)
    uint32_t idr = GPIOA->IDR;
    bool A = (idr & (1U << 4)) != 0;  // PA4
    bool B = (idr & (1U << 5)) != 0;  // PA5

    if (A != lastAR) {
        if (A == B) pulseR--; else pulseR++;  // ถ้าเดินหน้าแล้วเลขติดลบ ให้สลับ ++ กับ -- ตรงนี้
    } else if (B != lastBR) {
        if (A == B) pulseR++; else pulseR--;  // และสลับตรงนี้ด้วย
    }

    lastAR = A;
    lastBR = B;
}

// --- ฟังก์ชัน Setup ---
void setupEncoders() {
    pinMode(pinAL, INPUT_PULLUP);
    pinMode(pinBL, INPUT_PULLUP);
    pinMode(pinAR, INPUT_PULLUP);
    pinMode(pinBR, INPUT_PULLUP);

    // อ่านค่าเริ่มต้น
    uint32_t idr = GPIOA->IDR;
    lastAL = (idr & (1U << 0)) != 0;
    lastBL = (idr & (1U << 1)) != 0;
    lastAR = (idr & (1U << 4)) != 0;
    lastBR = (idr & (1U << 5)) != 0;

    // ใช้ CHANGE เพื่อความละเอียดสูงสุด (4x Decoding)
    attachInterrupt(digitalPinToInterrupt(pinAL), handleEncoderL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinBL), handleEncoderL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinAR), handleEncoderR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinBR), handleEncoderR, CHANGE);

    // ลด priority ของ Encoder EXTI interrupts ให้ต่ำกว่า Servo Timer
    // ARM Cortex-M: เลขยิ่งมาก = priority ยิ่งต่ำ (0 = สูงสุด)
    // Servo library ใช้ TIM3 ซึ่ง default priority = 0
    // → ตั้ง Encoder EXTI ที่ 4 ให้ Servo timer สามารถ preempt ได้
    NVIC_SetPriority(EXTI0_IRQn,   4);  // PA0 (pinAL)
    NVIC_SetPriority(EXTI1_IRQn,   4);  // PA1 (pinBL)
    NVIC_SetPriority(EXTI4_IRQn,   4);  // PA4 (pinAR)
    NVIC_SetPriority(EXTI9_5_IRQn, 4);  // PA5 (pinBR) — PA5–PA9 ใช้ EXTI9_5 ร่วมกัน
}

// --- ฟังก์ชันดึงข้อมูลไปใช้ ---
void getEncoderData(float &vL, float &vR, long &pL, long &pR, float dt) {
    // 1. Atomic Access: ป้องกัน Interrupt มาแทรกขณะอ่านค่า long
    noInterrupts();
    long currentL = pulseL;
    long currentR = pulseR;
    interrupts();

    // 2. คำนวณความเร็ว
    if (dt > 0) {
        float raw_vL = (float)(currentL - prevPulseL) / dt;
        float raw_vR = (float)(currentR - prevPulseR) / dt;

        // 3. Low-Pass Filter: กรองเลขกระโดด
        filtered_vL = (alpha * raw_vL) + ((1.0f - alpha) * filtered_vL);
        filtered_vR = (alpha * raw_vR) + ((1.0f - alpha) * filtered_vR);

        // ปัดเป็น 0 เมื่อหยุดนิ่ง
        if (fabsf(filtered_vL) < 0.5f) filtered_vL = 0;
        if (fabsf(filtered_vR) < 0.5f) filtered_vR = 0;

        vL = filtered_vL;
        vR = filtered_vR;
    } else {
        vL = 0; vR = 0;
    }

    // 4. ส่งค่ากลับและอัปเดตค่าเก่า
    pL = currentL;
    pR = currentR;
    prevPulseL = currentL;
    prevPulseR = currentR;
}