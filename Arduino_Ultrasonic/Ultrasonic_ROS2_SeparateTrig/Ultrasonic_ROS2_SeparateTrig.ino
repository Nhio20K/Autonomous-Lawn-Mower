#define TRIG1 3
#define ECHO1 4
#define TRIG2 5
#define ECHO2 6
#define TRIG3 7
#define ECHO3 8

#define NUM_SAMPLES 3      // ลดจาก 5
#define OFFSET 3.9
#define CHANGE_THRESHOLD 0.1
#define HEARTBEAT_INTERVAL 100

float lastDist[3] = {-1, -1, -1};
unsigned long lastSentTime = 0;

float getDistance(int trigPin, int echoPin) {
  float readings[NUM_SAMPLES];
  int validCount = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); // ลดจาก 5 เหลือ 2 พอครับ เพื่อความสะอาดของจังหวะ
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // ตามสเปก HC-SR04 สั่ง Trig แค่ 10us พอครับ
    digitalWrite(trigPin, LOW);

    // 🔥 จุดพลิกเกม: ลดเวลารอเสียงสะท้อนจาก 100ms (100000) เหลือแค่ 25ms (25000)
    // เพราะระยะ 4 เมตร เสียงเดินทางไปกลับใช้เวลาแค่ประมาณ 23ms ครับ นานกว่านี้คือบัคแน่นอน ไม่ต้องรอ!
    long duration = pulseIn(echoPin, HIGH, 25000); 

    if (duration > 0) {
      float dist = (duration / 2.0) * 0.0343 + OFFSET;
      if (dist >= 3.0 && dist <= 400.0) { // ระยะหว่าง 3 ซม. ถึง 4 เมตร
        readings[validCount++] = dist;
      }
    }
    // 🔥 ลบ delay(20); ทิ้งไปเลยครับ! หน่วงเวลาโดยใช่เหตุ
  }

  if (validCount == 0) return -1.0;

  // โค้ดเรียงลำดับ (Sorting) ข้อมูลเหมือนเดิม เพื่อหาค่ากลาง (Median) กรอง Noise
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = i + 1; j < validCount; j++) {
      if (readings[i] > readings[j]) {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  return readings[validCount / 2];
}


void setup() {
  Serial.begin(115200);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);
}

void loop() {
  float dist[3];
  dist[0] = getDistance(TRIG1, ECHO1);
  dist[1] = getDistance(TRIG2, ECHO2);
  dist[2] = getDistance(TRIG3, ECHO3);

  bool changed = false;
  for (int i = 0; i < 3; i++) {
    if (dist[i] < 0 && lastDist[i] >= 0) { changed = true; break; }
    if (dist[i] >= 0 && abs(dist[i] - lastDist[i]) >= CHANGE_THRESHOLD) { changed = true; break; }
  }

  bool timeout = millis() - lastSentTime >= HEARTBEAT_INTERVAL;

  if (changed || timeout) {
    Serial.print("U,");
    if (dist[0] < 0) Serial.print("-1"); else Serial.print(dist[0], 1);
    Serial.print(",");
    if (dist[1] < 0) Serial.print("-1"); else Serial.print(dist[1], 1);
    Serial.print(",");
    if (dist[2] < 0) Serial.println("-1"); else Serial.println(dist[2], 1);

    for (int i = 0; i < 3; i++) lastDist[i] = dist[i];
    lastSentTime = millis();
  }
}
