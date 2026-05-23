#include <Arduino.h>
#include "motor_control.h"
#include "Encoder.h"
#include "engine_control.h" // สำหรับระบบสตาร์ทเครื่องยนต์
#include "imu_control.h"    // สำหรับระบบ IMU BNO055
#include "battery_monitor.h" // สำหรับระบบวัดไฟ INA226

String inputBuffer = "";
bool emergencyMode = false;
uint32_t last_send = 0;
uint32_t last_cmd_received = 0;
const uint32_t HEARTBEAT_TIMEOUT = 1000;
bool data_stream_active = true; // ตัวแปรเปิด-ปิดการส่งข้อมูลรัวๆ

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    int first = cmd.indexOf(',');
    String header;
    if (first != -1) {
        header = cmd.substring(0, first);
    } else {
        header = cmd;
    }
    header.toUpperCase();

    if (header == "E" && first != -1) {
        int second = cmd.indexOf(',', first + 1);
        if (second != -1) {
            int state = cmd.substring(first + 1, second).toInt();
            int chk = cmd.substring(second + 1).toInt();
            if ((state + 69) == chk) {
                emergencyMode = (state == 1);
                if (emergencyMode) stop();
                last_cmd_received = millis();
            }
        }
    }

    if (header == "C" && first != -1) {
        int second = cmd.indexOf(',', first + 1);
        if (second == -1) return; // ป้องกัน second+1 เมื่อ second==-1
        int third = cmd.indexOf(',', second + 1);
        if (third != -1) {
            float vL = cmd.substring(first + 1, second).toFloat();
            float vR = cmd.substring(second + 1, third).toFloat();
            int receivedChk = cmd.substring(third + 1).toInt();
            // ใช้ roundf แทน truncation ป้องกัน checksum mismatch จาก float precision
            int calcChk = (int)roundf(vL * 100) + (int)roundf(vR * 100);
            if (calcChk == receivedChk) {
                if (emergencyMode) { stop(); return; }
                driveMotors(vL, vR);
                last_cmd_received = millis();
                // ไม่ reset data_stream_active ที่นี่
                // stream จะ resume ได้ด้วย R command เท่านั้น
            }
        }
    }

    // คำสั่งพิเศษ: S = หยุดส่ง stream เพื่อดึงค่า Calibration Profile 22 bytes
    if (header == "S") {
        data_stream_active = false;
        displayCalibrationData();
    }

    // คำสั่งพิเศษ: R = Re-init IMU และ resume data stream
    if (header == "R") {
        initIMU();
        data_stream_active = true; // resume stream หลัง re-init
    }

    // G = สตาร์ทเครื่องยนต์ (Go)
    if (header == "G") {
        startEngine();
        Serial1.println("ENGINE: Starting...");
    }

    // X = ดับเครื่องยนต์ (eXtinguish)
    if (header == "X") {
        powerOff();
        Serial1.println("ENGINE: Off.");
    }
}

void setup() {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    pinMode(PB14, INPUT_PULLUP);
    pinMode(PC14, OUTPUT);
    pinMode(PC13, OUTPUT);

    Serial1.begin(115200);

    initMotors();       // เริ่มระบบมอเตอร์ล้อ
    setupEncoders();    // เริ่มระบบ Encoder
    initIMU();          // เริ่มระบบ IMU BNO055
    initEngineSystem(); // เริ่มระบบสตาร์ทเครื่องยนต์
    initBattery();      // เริ่มระบบวัดไฟแบตเตอรี่

    last_cmd_received = millis();
    last_send = millis(); // ป้องกัน dt ผิดรอบแรก
}

void loop() {
    // 1. รับคำสั่ง Serial
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                processCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
            if (inputBuffer.length() > 50) inputBuffer = "";
        }
    }

    // 2. Safety Heartbeat
    if (!emergencyMode && (millis() - last_cmd_received > HEARTBEAT_TIMEOUT)) {
        stop();
    }

    // 3. อัปเดต IMU (fast path ทุก loop, slow path ทุก 100ms — ไม่บล็อก loop นาน)
    updateIMU();

    // 4. ระบบจัดการเครื่องยนต์ (ใช้ imu_data ที่อ่านไว้แล้วจากข้อ 3 — ไม่เรียก I2C ซ้ำ)
    updateStarterStatus();

    // 5. ส่งข้อมูลกลับ Pi ทุก 100ms (เฉพาะตอนที่ไม่ได้หยุดการส่ง)
    if (data_stream_active && (millis() - last_send >= 100)) {
        uint32_t now = millis();
        float dt = (now - last_send) / 1000.0f;
        last_send = now;

        // ส่งข้อมูล Encoder
        float vL, vR; long pL, pR;
        getEncoderData(vL, vR, pL, pR, dt);
        Serial1.print("D,"); Serial1.print(vL, 2); Serial1.print(",");
        Serial1.print(vR, 2); Serial1.print(","); Serial1.print(pL); Serial1.print(",");
        Serial1.println(pR);

        // ส่งข้อมูล IMU BNO055 Heading
        Serial1.print("H,"); Serial1.println(imu_data.yaw, 2);

        // ส่งข้อมูล IMU Full Data (สำหรับ ROS2 EKF)
        Serial1.print("I,");
        Serial1.print(imu_data.accX, 3);  Serial1.print(",");
        Serial1.print(imu_data.accY, 3);  Serial1.print(",");
        Serial1.print(imu_data.accZ, 3);  Serial1.print(",");
        Serial1.print(imu_data.gyroX, 4); Serial1.print(",");
        Serial1.print(imu_data.gyroY, 4); Serial1.print(",");
        Serial1.print(imu_data.gyroZ, 4); Serial1.print(",");
        Serial1.print(imu_data.qX, 4);    Serial1.print(",");
        Serial1.print(imu_data.qY, 4);    Serial1.print(",");
        Serial1.print(imu_data.qZ, 4);    Serial1.print(",");
        Serial1.print(imu_data.qW, 4);    Serial1.print(",");
        Serial1.print(imu_data.sys);       Serial1.print(",");
        Serial1.print(imu_data.gyro);      Serial1.print(",");
        Serial1.print(imu_data.accel);     Serial1.print(",");
        Serial1.println(imu_data.mag);

        // ส่งข้อมูลแบตเตอรี่ (มี rate limit + bat_ok guard ภายในฟังก์ชัน)
        updateBattery();
    }

    // 6. สวิตช์ Manual/Auto
    bool isManual = (digitalRead(PB14) == LOW);
    digitalWrite(PC14, isManual ? LOW : HIGH);
    digitalWrite(PC13, isManual ? HIGH : LOW);
}