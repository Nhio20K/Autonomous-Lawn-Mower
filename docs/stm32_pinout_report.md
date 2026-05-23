# STM32 Bluepill Pinout Report (Mower Project)

เอกสารสรุปการใช้งานขา (Pin Assignment) ทั้งหมดของหุ่นยนต์ตัดหญ้า อัปเดตล่าสุดตามโค้ดจริงใน Workspace

## 🔴 ขาที่ถูกใช้งานแล้ว (IN USE - DO NOT REASSIGN)

| หมวดหมู่ | ขา (Pin) | หน้าที่ (Function) | หมายเหตุ |
| :--- | :--- | :--- | :--- |
| **Encoders** | PA0, PA1 | Encoder ล้อซ้าย | ใช้ Interrupt (CHANGE) |
| | PA4, PA5 | Encoder ล้อขวา | ใช้ Interrupt (CHANGE) |
| **Motors** | PB4, PB5 | คุมมอเตอร์ตีนตะขาบ | ใช้ไลบรารี Servo (PWM) |
| **Engine** | PB3 | คุมคันเร่งเครื่องยนต์ | ใช้ไลบรารี Servo (PWM) |
| **I2C Bus (1)** | PB6, PB7 | **BNO055** (IMU) + **INA226** (Battery) | I2C1 (SCL: PB6, SDA: PB7) |
| **Serial** | PA9, PA10 | เชื่อมต่อกับ Raspberry Pi | UART1 (TX: PA9, RX: PA10) |
| **UI/Control** | PB14 | สวิตช์สลับโหมด Manual/Auto | Input Pull-up |
| | PC13 | ไฟแสดงสถานะโหมด **Manual** | Output (Built-in LED) |
| | PC14 | ไฟแสดงสถานะโหมด **Auto** | Output (External LED) |

---

## 🟢 ขาที่ยังว่าง (FREE PINS)

| ขา (Pin) | ฟังก์ชันที่รองรับ (Peripherals) | คำแนะนำในการใช้งาน |
| :--- | :--- | :--- |
| **PB15** | PWM, SPI2, ADC | **ว่าง** (เดิมเป็น Relay สตาร์ทเครื่องยนต์ แต่ปลดออกแล้ว) |
| **PB8, PB9** | **I2C2** (SCL/SDA) | **แนะนำที่สุด** สำหรับติดเซนเซอร์ I2C เพิ่ม (เช่น จอ OLED) |
| **PA2, PA3** | UART2 (TX/RX), ADC | ใช้ต่อเซนเซอร์ที่ส่งข้อมูล Serial หรือ Analog |
| **PB0, PB1** | **ADC** (Analog In), PWM | เหมาะสำหรับเซนเซอร์ Analog เช่น วัดแรงดันเพิ่ม |
| **PB10, PB11** | UART3, I2C2 | ขาสำรองสำหรับ Serial หรือ I2C |
| **PA6, PA7** | PWM, ADC | ใช้คุม Servo เพิ่มเติม หรืออ่านค่า Analog |
| **PB12, PB13** | SPI2, UART3 | ใช้ต่อโมดูล SPI เช่น NRF24L01 หรือ SD Card |

---

## ⚠️ สเปคและข้อควรระวัง
1. **5V Tolerance**: 
   - ขาส่วนใหญ่รับ 5V ได้ (เช่น PB3-PB15, PA9-PA10)
   - ❌ **PA0, PA1, PA4, PA5 (Encoder) ไม่ได้รับ 5V (3.3V Only)** โปรดระวังความเสียหายหากต่อไฟเกิน
2. **PA11, PA12**: สองขานี้จะถูกจองถ้ามีการใช้ USB บนบอร์ด (D-/D+) แนะนำให้เลี่ยงถ้าไม่จำเป็น
3. **PB2 (BOOT1)**: ถูกผูกไว้กับระบบ Bootloader ไม่ควรนำมาใช้เป็น IO ทั่วไป
4. **SWD (PA13, PA14)**: ใช้สำหรับลงโปรแกรม (Flash) ควรปล่อยว่างไว้เพื่อความสะดวกในการแก้โค้ด
