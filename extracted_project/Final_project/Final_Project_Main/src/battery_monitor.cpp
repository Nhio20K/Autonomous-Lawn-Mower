#include "battery_monitor.h"

#define I2C_ADDRESS_INA226 0x40

INA226_WE ina226 = INA226_WE(I2C_ADDRESS_INA226);
bool bat_ok = false;

void initBattery() {
    if(!ina226.init()){
        Serial1.println("[ERROR] INA226 not detected! Battery monitoring disabled.");
        bat_ok = false;
    } else {
        // ตั้งค่าสำหรับ Shunt 50A / 75mV (0.0015 Ohm)
        ina226.setResistorRange(0.0015, 50.0);

        // ใช้ Hardware Averaging แทน delay() loop เพื่อไม่บล็อก CPU
        // AVERAGE_16 = เฉลี่ย 16 ตัวอย่างภายใน IC → ค่านิ่งโดยไม่ต้อง delay ใน code
        ina226.setAverage(INA226_AVERAGE_16);

        ina226.waitUntilConversionCompleted();
        bat_ok = true;
        Serial1.println("[OK] Battery Monitor Ready (HW AVG x16)");
    }
}

void updateBattery() {
    if (!bat_ok) return;  // ข้าม I2C ถ้า sensor ไม่ได้ต่อ

    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 500) return; // ส่งข้อมูลทุกๆ 0.5 วินาที
    lastUpdate = millis();

    // อ่านค่าครั้งเดียว — INA226 ทำ averaging ใน hardware แล้ว (ไม่ต้อง loop + delay)
    float volt = ina226.getBusVoltage_V();
    float curr = ina226.getCurrent_mA();

    // ใช้ Factor ที่จูนไว้จาก ESP32
    float corrected_volt_V = volt * 1.04; 
    float corrected_current_A = (curr / 1000.0) * 0.915;

    // ส่งออกทาง Serial1 (ตรงกับพอร์ตที่ Pi รับ)
    Serial1.print("B,");
    Serial1.print(corrected_volt_V, 2);
    Serial1.print(",");
    Serial1.println(corrected_current_A, 2);
}
