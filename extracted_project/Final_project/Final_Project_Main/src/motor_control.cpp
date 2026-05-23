#include "motor_control.h"

Servo motorL, motorR;

void initMotors() {
    motorL.attach(MOTOR_L_PIN);
    motorR.attach(MOTOR_R_PIN);
    stopMotors();
}

// แปลง m/s → PWM µs พร้อม deadband + minimum offset compensation
// ถ้า |v| < DEADBAND_MS  → 1500µs (หยุดนิ่ง)
// ถ้า |v| >= DEADBAND_MS → กระโดดไป (1500 ± MIN_PWM_OFFSET) แล้ว scale ขึ้นถึง (1000/2000)
static int msToPWM(float v_ms) {
    if (fabsf(v_ms) < DEADBAND_MS) return 1500;
    float sign   = (v_ms > 0.0f) ? 1.0f : -1.0f;
    float ratio  = fabsf(v_ms) / MAX_SPEED_MS;
    int   offset = MIN_PWM_OFFSET + (int)(ratio * (500 - MIN_PWM_OFFSET));
    return 1500 + (int)(sign * offset);
}

void driveMotors(float vL_ms, float vR_ms) {
    vL_ms = constrain(vL_ms, -MAX_SPEED_MS, MAX_SPEED_MS);
    vR_ms = constrain(vR_ms, -MAX_SPEED_MS, MAX_SPEED_MS);

    motorL.writeMicroseconds(constrain(msToPWM(vL_ms), 1000, 2000));
    motorR.writeMicroseconds(constrain(msToPWM(vR_ms), 1000, 2000));
}

void stopMotors() {
    motorL.writeMicroseconds(1500);
    motorR.writeMicroseconds(1500);
}

void stop() { stopMotors(); }
