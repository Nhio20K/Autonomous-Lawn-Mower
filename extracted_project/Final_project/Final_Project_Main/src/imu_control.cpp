#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "imu_control.h"

// กำหนด Pin I2C สำหรับ STM32 Bluepill
// PB6 = SCL, PB7 = SDA
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

OrientationData imu_data;
bool imu_ok = false;

void initIMU() {
    imu_ok = false;  // Reset flag ทุกครั้งที่ init ใหม่
    Serial1.println("--- BNO055 Initializing (PB6:SCL, PB7:SDA) ---");

    // เรียก Wire.begin() ครั้งเดียว ป้องกัน I2C bus hang เมื่อ R command re-init
    static bool wire_initialized = false;
    if (!wire_initialized) {
        Wire.setSDA(PB7);
        Wire.setSCL(PB6);
        Wire.begin();
        wire_initialized = true;
    }

    if(!bno.begin()) {
        Serial1.println("[ERROR]: BNO055 not detected! System will continue but IMU data will be 0.");
        return;
    }

    delay(1000);
    bno.setExtCrystalUse(true);

    delay(500);
    // 1. เข้าโหมด CONFIG
    bno.setMode((adafruit_bno055_opmode_t)0x00);
    delay(100);

    // เตรียมข้อมูล 22 bytes
    int16_t vals[11] = {-25, -1, -39, 364, 166, -72, -4, -2, -2, 1000, 557};
    uint8_t raw_vals[22];
    for(int i=0; i<11; i++) {
        raw_vals[i*2]   = vals[i] & 0xFF;
        raw_vals[i*2+1] = (vals[i] >> 8) & 0xFF;
    }

    // 2. เขียนครั้งที่ 1
    Wire.beginTransmission(0x28);
    Wire.write(0x55);
    for(int i=0; i<22; i++) Wire.write(raw_vals[i]);
    Wire.endTransmission();
    delay(100);

    // 3. ลองสลับไปโหมดอื่นแป๊บนึงแล้วกลับมาเขียนซ้ำ (เทคนิคกระตุ้น)
    bno.setMode((adafruit_bno055_opmode_t)0x08); // IMU Mode
    delay(100);
    bno.setMode((adafruit_bno055_opmode_t)0x00); // กลับมา CONFIG
    delay(100);

    // 4. เขียนครั้งที่ 2
    Wire.beginTransmission(0x28);
    Wire.write(0x55);
    for(int i=0; i<22; i++) Wire.write(raw_vals[i]);
    Wire.endTransmission();
    delay(100);

    // 5. เข้าโหมด NDOF (0x0C) เพื่อเริ่มใช้งาน
    bno.setMode((adafruit_bno055_opmode_t)0x0C);
    delay(200);

    imu_ok = true;
    Serial1.println("✅ [DONE]: BNO055 Calibration Profile Double-Forced!");
    Serial1.println("BNO055 Connected Successfully!");
}

void updateIMU() {
    if (!imu_ok) return;

    // --- Fast path: อ่านเฉพาะ orientation ทุก loop (~400µs) ---
    sensors_event_t event;
    bno.getEvent(&event);
    imu_data.yaw   = event.orientation.x;
    imu_data.pitch = event.orientation.y;
    imu_data.roll  = event.orientation.z;

    // --- Slow path: อ่าน full data (Quat, Accel, Gyro, Calib) ทุก 100ms เท่านั้น ---
    // ลด I2C load จาก 5 transactions/loop → 1 transaction/loop (ทั่วไป)
    static uint32_t last_full_read = 0;
    uint32_t now = millis();
    if (now - last_full_read < 100) return;
    last_full_read = now;

    // ดึงค่า Quaternion
    imu::Quaternion quat = bno.getQuat();
    imu_data.qW = quat.w();
    imu_data.qX = quat.x();
    imu_data.qY = quat.y();
    imu_data.qZ = quat.z();

    // ดึงค่าความเร่ง (Linear Acceleration - ไม่มีแรงโน้มถ่วง)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_data.accX = accel.x();
    imu_data.accY = accel.y();
    imu_data.accZ = accel.z();

    // ดึงค่าความเร็วเชิงมุม (Gyroscope)
    imu::Vector<3> gyroVec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_data.gyroX = gyroVec.x();
    imu_data.gyroY = gyroVec.y();
    imu_data.gyroZ = gyroVec.z();

    // สถานะ Calibration
    bno.getCalibration(&imu_data.sys, &imu_data.gyro, &imu_data.accel, &imu_data.mag);
}

// ฟังก์ชันสำหรับดึงค่า Calibration 22 bytes แบบบังคับอ่าน (Direct Register Read)
void displayCalibrationData() {
    if (!imu_ok) {
        Serial1.println("[WARN] IMU not available — cannot read calibration.");
        return;
    }
    Serial1.println("\n--- BNO055 RAW CALIBRATION DATA (22 BYTES) ---");
    Serial1.println("Reading directly from registers 0x55 - 0x6A...");

    // ต้องเข้าโหมด CONFIG (0x00) ก่อนถึงจะอ่าน Register กลุ่มนี้ได้
    adafruit_bno055_opmode_t last_mode = bno.getMode();
    bno.setMode((adafruit_bno055_opmode_t)0x00);
    delay(100);

    uint8_t data[22] = {0};  // init เป็น 0 ป้องกัน garbage ถ้า Wire ส่งไม่ครบ
    Wire.beginTransmission(0x28);
    Wire.write(0x55);
    Wire.endTransmission(false);
    Wire.requestFrom(0x28, (uint8_t)22);

    int bytesRead = 0;
    for (int i = 0; i < 22; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
            bytesRead++;
        }
    }
    if (bytesRead < 22) {
        Serial1.print("[WARN] Calibration read incomplete: got ");
        Serial1.print(bytesRead);
        Serial1.println("/22 bytes. Data may be invalid.");
    }

    int16_t accX = (int16_t)data[0]  | ((int16_t)data[1]  << 8);
    int16_t accY = (int16_t)data[2]  | ((int16_t)data[3]  << 8);
    int16_t accZ = (int16_t)data[4]  | ((int16_t)data[5]  << 8);
    int16_t gyrX = (int16_t)data[6]  | ((int16_t)data[7]  << 8);
    int16_t gyrY = (int16_t)data[8]  | ((int16_t)data[9]  << 8);
    int16_t gyrZ = (int16_t)data[10] | ((int16_t)data[11] << 8);
    int16_t magX = (int16_t)data[12] | ((int16_t)data[13] << 8);
    int16_t magY = (int16_t)data[14] | ((int16_t)data[15] << 8);
    int16_t magZ = (int16_t)data[16] | ((int16_t)data[17] << 8);
    int16_t accR = (int16_t)data[18] | ((int16_t)data[19] << 8);
    int16_t magR = (int16_t)data[20] | ((int16_t)data[21] << 8);

    Serial1.print("Copy these values: ");
    Serial1.print(accX); Serial1.print(","); Serial1.print(accY); Serial1.print(","); Serial1.print(accZ); Serial1.print(",");
    Serial1.print(gyrX); Serial1.print(","); Serial1.print(gyrY); Serial1.print(","); Serial1.print(gyrZ); Serial1.print(",");
    Serial1.print(magX); Serial1.print(","); Serial1.print(magY); Serial1.print(","); Serial1.print(magZ); Serial1.print(",");
    Serial1.print(accR); Serial1.print(","); Serial1.print(magR);
    Serial1.println("\n--------------------------------------------");

    // กลับสู่โหมดเดิม
    bno.setMode(last_mode);
    delay(100);
    Serial1.println("Type 'R' to resume streaming.");
}