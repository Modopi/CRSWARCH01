#include <SoftwareSerial.h>
#include <SPI.h>
#include <MFRC522.h>
#include <math.h>

//zs 040
SoftwareSerial hc06(2, 3);

// ── RFID (MFRC522) ──
// 결선: SDA→D10, SCK→D13, MOSI→D11, MISO→D12, RST→D9, 3.3V, GND  (IRQ 미사용)
const uint8_t RFID_SS_PIN     = 10;
const uint8_t RFID_RST_PIN    = 9;
const uint8_t RFID_SIGNAL_PIN = 4;       // 카드 인식 시 HIGH 출력
const unsigned long RFID_HOLD_MS = 2000; // 인식 후 HIGH 유지 시간 (ms)
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
unsigned long rfidHighUntil = 0;

// ── 파라미터 ──
const float HEAD_MASS    = 6.0;    // 머리+헬멧 추정 질량 (kg)
const float G            = 9.807;  // m/s²
const float IMPACT_THRESH = 16;   // 충격 판정 임계 (g)

// ── 이전 프레임 기억 ──
float prevAx = 0, prevAy = 0, prevAz = 1.0;  // 이전 가속도 (g)
float prevMag = 1.0;                           // 이전 가속도 크기
unsigned long prevTime = 0;
bool firstFrame = true;

// ── 피크 추적 ──
float peakG = 0;
float peakForce = 0;

// ── CSV 파싱 ──
bool parseCSV(const String &line, float *vals, int count) {
  int idx = 0, start = 0;
  for (int i = 0; i <= (int)line.length() && idx < count; i++) {
    if (i == (int)line.length() || line.charAt(i) == ',') {
      vals[idx++] = line.substring(start, i).toFloat();
      start = i + 1;
    }
  }
  return idx == count;
}

void setup() {
  Serial.begin(9600);
  hc06.begin(9600);
  prevTime = millis();

  // RFID 초기화
  SPI.begin();
  rfid.PCD_Init();
  rfid.PCD_SetAntennaGain(rfid.RxGain_max);  // 안테나 게인 최대
  rfid.PCD_AntennaOn();                       // 명시적으로 한번 더 ON
  pinMode(RFID_SIGNAL_PIN, OUTPUT);
  digitalWrite(RFID_SIGNAL_PIN, LOW);

  // 진단: RC522 펌웨어 버전 출력 (정상=0x91/0x92, 비정상=0x00/0xFF)
  Serial.print(F("# RC522 ver: "));
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.println(v, HEX);
  rfid.PCD_DumpVersionToSerial();

  // 헤더 — TUI는 "IMU,"로 시작하지 않는 줄을 무시함
  Serial.println(F("# Smart Helmet Impact Monitor — CSV stream"));
  Serial.println(F("# fields: IMU,ax,ay,az,gx,gy,gz,temp,light,aMag,aNet,force,impulse,jerk,roll,pitch,gyroMag,peakG,peakForce,impact"));
  Serial.println(F("# event:  RFID,<UID_HEX>"));
}

// 카드가 새로 잡혔는지 확인하고, 잡혔으면 HIGH 타이머 갱신
// 출력 형식: RFID,<UID_HEX>\n   (TUI에서 파싱)
void pollRFID() {
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    rfidHighUntil = millis() + RFID_HOLD_MS;
    Serial.print(F("RFID,"));
    for (byte i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] < 0x10) Serial.print('0');
      Serial.print(rfid.uid.uidByte[i], HEX);
    }
    Serial.println();
    rfid.PICC_HaltA();
  }
  digitalWrite(RFID_SIGNAL_PIN, (millis() < rfidHighUntil) ? HIGH : LOW);
}

void loop() {
  // RFID는 매 루프 폴링 (BT 데이터 유무와 무관)
  pollRFID();

  if (!hc06.available()) return;

  String line = hc06.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // ax, ay, az, gx, gy, gz, temp, light
  float v[8];
  if (!parseCSV(line, v, 8)) return;

  float ax = v[0], ay = v[1], az = v[2];   // g
  float gx = v[3], gy = v[4], gz = v[5];   // °/s
  float temp = v[6];                         // °C
  int   light = (int)v[7];                   // 0..1023 (raw ADC)

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;     // 초
  if (dt <= 0) dt = 0.1;
  prevTime = now;

  // ━━━ 1. 가속도 크기 (g) ━━━
  float aMag = sqrt(ax*ax + ay*ay + az*az);

  // ━━━ 2. 순 가속도 — 중력 제거 (g) ━━━
  float aNet = fabs(aMag - 1.0);

  // ━━━ 3. 충격력 F = m · a_net (N) ━━━
  float aNet_ms2 = aNet * G;
  float force = HEAD_MASS * aNet_ms2;

  // ━━━ 4. 충격량 J = F · Δt (N·s) ━━━
  float impulse = force * dt;

  // ━━━ 5. 저크 (g/s) — 가속도 변화율 ━━━
  float jerk = 0;
  if (!firstFrame) {
    float dAx = ax - prevAx;
    float dAy = ay - prevAy;
    float dAz = az - prevAz;
    jerk = sqrt(dAx*dAx + dAy*dAy + dAz*dAz) / dt;
  }

  // ━━━ 6. 자세각 (°) — 가속도 기반 ━━━
  float roll  = atan2(ay, az) * 180.0 / M_PI;
  float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;

  // ━━━ 7. 각속도 크기 (°/s) ━━━
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  // ━━━ 피크 갱신 ━━━
  if (aMag > peakG)     peakG = aMag;
  if (force > peakForce) peakForce = force;

  // ━━━ 충격 판정 ━━━
  bool impact = (aMag >= IMPACT_THRESH);

  // ━━━━━━ 출력 (CSV, TUI에서 파싱) ━━━━━━
  // 형식: IMU,ax,ay,az,gx,gy,gz,temp,light,aMag,aNet,force,impulse,jerk,roll,pitch,gyroMag,peakG,peakForce,impact
  Serial.print(F("IMU,"));
  Serial.print(ax, 3);       Serial.print(',');
  Serial.print(ay, 3);       Serial.print(',');
  Serial.print(az, 3);       Serial.print(',');
  Serial.print(gx, 2);       Serial.print(',');
  Serial.print(gy, 2);       Serial.print(',');
  Serial.print(gz, 2);       Serial.print(',');
  Serial.print(temp, 1);     Serial.print(',');
  Serial.print(light);       Serial.print(',');
  Serial.print(aMag, 3);     Serial.print(',');
  Serial.print(aNet, 3);     Serial.print(',');
  Serial.print(force, 2);    Serial.print(',');
  Serial.print(impulse, 4);  Serial.print(',');
  Serial.print(jerk, 2);     Serial.print(',');
  Serial.print(roll, 1);     Serial.print(',');
  Serial.print(pitch, 1);    Serial.print(',');
  Serial.print(gyroMag, 1);  Serial.print(',');
  Serial.print(peakG, 3);    Serial.print(',');
  Serial.print(peakForce, 1); Serial.print(',');
  Serial.println(impact ? 1 : 0);

  // ── 다음 프레임 준비 ──
  prevAx = ax; prevAy = ay; prevAz = az;
  prevMag = aMag;
  firstFrame = false;
}
