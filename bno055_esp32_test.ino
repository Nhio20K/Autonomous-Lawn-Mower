#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imu_orientation.h>

/*
   การต่อสาย ESP32:
   BNO055 Vin   ->  ESP32 3.3V (หรือ 5V ถ้ามี Regulator)
   BNO055 GND   ->  ESP32 GND
   BNO055 SDA   ->  ESP32 GPIO 21
   BNO055 SCL   ->  ESP32 GPIO 22
*/

// สร้าง Object สำหรับ BNO055 (Address ปกติคือ 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // รอให้ Serial Monitor เปิด

  Serial.println("======================================");
  Serial.println("   BNO055 ESP32 PROOF OF CONCEPT      ");
  Serial.println("======================================");

  /* เริ่มต้นใช้งาน Sensor */
  if(!bno.begin()) {
    Serial.print("ERROR: No BNO055 detected! Check wiring or I2C Addr (0x28/0x29)");
    while(1);
  }

  delay(1000);
    
  /* ใช้ External Crystal เพื่อความแม่นยำสูงสุด (ถ้าบอร์ดมี) */
  bno.setExtCrystalUse(true);
}

void loop() {
  /* 1. อ่านค่า Orientation (Euler Angles) */
  sensors_event_t event;
  bno.getEvent(&event);

  /* 2. อ่านสถานะการ Calibration (0-3, 3 คือแม่นยำสุด) */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* แสดงผลออก Serial Monitor */
  Serial.print("Heading: ");
  Serial.print(event.orientation.x, 2); // 0-360 องศา
  Serial.print(" | Pitch: ");
  Serial.print(event.orientation.y, 2);
  Serial.print(" | Roll: ");
  Serial.print(event.orientation.z, 2);

  /* แสดงสถานะ Calibration */
  Serial.print("  [CAL] Sys:");
  Serial.print(system);
  Serial.print(" G:");
  Serial.print(gyro);
  Serial.print(" A:");
  Serial.print(accel);
  Serial.print(" M:");
  Serial.println(mag);

  /* 
     TIP การ Calibrate:
     - Magnetometer (M): วาดเลข 8 ในอากาศ
     - Gyro (G): วาง Sensor นิ่งๆ สัก 1-2 วินาที
     - Accel (A): พลิก Sensor ไปมาในแนวแกนต่างๆ (6 ทิศทาง)
  */

  delay(100);
}
