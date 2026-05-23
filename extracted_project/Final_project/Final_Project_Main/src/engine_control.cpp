#include "engine_control.h"
#include "imu_control.h"

Servo engineServo; // ยังใช้อยู่สำหรับคุม Throttle เครื่องยนต์

bool isStarting = false;
uint32_t startTimer = 0;
const float VIBRATION_THRESHOLD = 5.0f; 

void initEngineSystem() {
    engineServo.attach(PIN_THROTTLE_SERVO);
    // relayRC (PB15) ถูกปลดออกแล้ว — ไม่ได้ใช้งาน
    
    // เริ่มต้นให้ดับเครื่องทันทีเพื่อความปลอดภัย
    powerOff(); 
    engineServo.writeMicroseconds(1100); 
}

void setThrottle(int percent) {
    percent = constrain(percent, 0, 100);
    int pwm = map(percent, 0, 100, 1100, 1900);
    engineServo.writeMicroseconds(pwm);
}

void powerOn() {
    // relayRC ถูกปลดออกแล้ว — ไม่มีการทำงานในส่วนนี้
}

void powerOff() {
    // relayRC ถูกปลดออกแล้ว — ไม่มีการทำงานในส่วนนี้
    isStarting = false;
}

void startEngine() {
    // relayRC ถูกปลดออกแล้ว — ไม่มีการทำงานในส่วนนี้
    isStarting = true;
    startTimer = millis();
}

void updateStarterStatus() {
    if (!isStarting) return;

    // ใช้ imu_data ที่อ่านไว้แล้วจาก updateIMU() ใน main loop (ไม่เรียกซ้ำ)
    float totalVibration = abs(imu_data.accX) + abs(imu_data.accY) + abs(imu_data.accZ);

    if (totalVibration > VIBRATION_THRESHOLD || (millis() - startTimer > 10000)) {
        // เมื่อเครื่องติดแล้ว ให้กลับไปสถานะ Power On (Relay OFF เพื่อให้เครื่องวิ่งต่อได้)
        powerOn(); 
        isStarting = false;
    }
}