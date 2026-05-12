#include <Wire.h>
#include <SoftwareSerial.h>

//정품보드(빨강+파랑불)이 슬레이브
SoftwareSerial hc06(2, 3);  // RX, TX

const uint8_t MPU_ADDR = 0x68;
const uint8_t LIGHT_PIN = A0;   // 조도 센서 (CDS)

void mpuWrite(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void mpuRead(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

int16_t combine(uint8_t h, uint8_t l) {
  return (int16_t)((h << 8) | l);
}

void setup() {
  Serial.begin(9600);
  hc06.begin(9600);
  Wire.begin();

  // MPU-6050 초기화
  mpuWrite(0x6B, 0x00);   // 슬립 해제
  delay(100);
  mpuWrite(0x1C, 0x00);   // ±2g
  mpuWrite(0x1B, 0x00);   // ±250°/s
  mpuWrite(0x1A, 0x03);   // DLPF ~44Hz

  Serial.println("Slave ready");
}

void loop() {
  // --- MPU-6050 읽기 ---
  uint8_t buf[14];
  mpuRead(0x3B, buf, 14);

  float ax = combine(buf[0],  buf[1])  / 16384.0;
  float ay = combine(buf[2],  buf[3])  / 16384.0;
  float az = combine(buf[4],  buf[5])  / 16384.0;
  float temp = combine(buf[6], buf[7]) / 340.0 + 36.53;
  float gx = combine(buf[8],  buf[9])  / 131.0;
  float gy = combine(buf[10], buf[11]) / 131.0;
  float gz = combine(buf[12], buf[13]) / 131.0;

  // --- 조도 센서 읽기 (0~1023) ---
  int light = analogRead(LIGHT_PIN);

  // --- 블루투스로 전송 (CSV 형식) ---
  // 형식: ax,ay,az,gx,gy,gz,temp,light\n
  char line[96];
  dtostrf(ax, 1, 3, line);
  strcat(line, ",");
  char tmp[12];

  dtostrf(ay, 1, 3, tmp); strcat(line, tmp); strcat(line, ",");
  dtostrf(az, 1, 3, tmp); strcat(line, tmp); strcat(line, ",");
  dtostrf(gx, 1, 2, tmp); strcat(line, tmp); strcat(line, ",");
  dtostrf(gy, 1, 2, tmp); strcat(line, tmp); strcat(line, ",");
  dtostrf(gz, 1, 2, tmp); strcat(line, tmp); strcat(line, ",");
  dtostrf(temp, 1, 1, tmp); strcat(line, tmp); strcat(line, ",");
  itoa(light, tmp, 10); strcat(line, tmp);

  hc06.println(line);     // 블루투스로 송신
  Serial.println(line);   // 디버깅용 시리얼 모니터

  // --- 마스터→슬레이브 명령 수신 (선택) ---
  if (hc06.available()) {
    Serial.write(hc06.read());
  }

  delay(100);  // ~10Hz 전송 주기
}
