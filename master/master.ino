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
const float IMPACT_THRESH = 2;     // 충격 판정 임계 — 수평 선형가속도 (g)

// ── 이전 프레임 기억 ──
float prevHx = 0, prevHy = 0, prevHz = 0;  // 이전 수평 선형가속도 (g)
unsigned long prevTime = 0;
bool firstFrame = true;

// ── 중력 벡터 추정 (상보 필터, 센서 프레임 기준, g 단위) ──
//  자이로로 중력 방향을 회전시켜 예측하고, 가속도로 저주파 보정.
//  헬멧이 회전/기울어져도 중력 방향을 따라가므로 "z축=중력" 가정에 의존하지 않음.
float gravX = 0, gravY = 0, gravZ = 1.0;
const float COMP_ALPHA = 0.98;       // 자이로 예측 신뢰 비중 (나머지는 가속도 보정)
const float DEG2RAD    = 0.0174532925f;

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
  Serial.begin(115200);   // USB → PC(TUI). 하드웨어 UART라 115200 안정 (긴 줄 블로킹 최소화)
  hc06.begin(38400);      // BT ← slave. SoftwareSerial 안정 상한 = 38400
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
  Serial.println(F("# fields: IMU,ax,ay,az,gx,gy,gz,temp,light,fsr,alcohol,aMag,aNet,aHoriz,force,impulse,jerk,roll,pitch,gyroMag,peakG,peakForce,impact"));
  Serial.println(F("# event:  RFID,<UID_HEX>"));
  Serial.println(F("# event:  IMPACT,peakG,peakForce,impulse,durMs,peakJerk  (slave 고속 적분 결과)"));
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

  // ── 충격 이벤트 (slave가 고속 적분한 결과) → PC로 그대로 전달 ──
  //  slave 형식: E,peakG,peakForce,impulse,durMs,peakJerk
  //  TUI 형식:   IMPACT,peakG,peakForce,impulse,durMs,peakJerk
  if (line.startsWith("E,")) {
    Serial.print(F("IMPACT,"));
    Serial.println(line.substring(2));
    return;
  }

  // ax, ay, az, gx, gy, gz, temp, light, fsr, alcohol
  float v[10];
  if (!parseCSV(line, v, 10)) return;

  float ax = v[0], ay = v[1], az = v[2];   // g
  float gx = v[3], gy = v[4], gz = v[5];   // °/s
  float temp = v[6];                         // °C
  int   light   = (int)v[7];                 // 0..1023 (raw ADC)
  int   fsr     = (int)v[8];                 // 0..1023 (raw ADC, FSR402)
  int   alcohol = (int)v[9];                 // 0..1023 (raw ADC, MQ-3)

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;     // 초
  if (dt <= 0) dt = 0.1;
  prevTime = now;

  // ━━━ 1. 가속도 크기 (g) ━━━
  float aMag = sqrt(ax*ax + ay*ay + az*az);

  // ━━━ 2. 중력 벡터 추정 — 상보 필터 (자이로 회전 예측 + 가속도 보정) ━━━
  if (firstFrame) {
    gravX = ax; gravY = ay; gravZ = az;          // 시작 자세로 초기화
  } else {
    float wx = gx * DEG2RAD, wy = gy * DEG2RAD, wz = gz * DEG2RAD;  // rad/s
    // 센서 프레임에서 본 중력은 -ω 로 회전:  g' = g − (ω × g)·dt
    float gxp = gravX - (wy*gravZ - wz*gravY) * dt;
    float gyp = gravY - (wz*gravX - wx*gravZ) * dt;
    float gzp = gravZ - (wx*gravY - wy*gravX) * dt;
    gravX = COMP_ALPHA*gxp + (1.0f-COMP_ALPHA)*ax;
    gravY = COMP_ALPHA*gyp + (1.0f-COMP_ALPHA)*ay;
    gravZ = COMP_ALPHA*gzp + (1.0f-COMP_ALPHA)*az;
  }
  float gN = sqrt(gravX*gravX + gravY*gravY + gravZ*gravZ);
  if (gN < 1e-3) gN = 1.0;
  float gHx = gravX/gN, gHy = gravY/gN, gHz = gravZ/gN;  // 중력 단위벡터(=지면 법선)

  // ━━━ 3. 선형 가속도 — 중력 벡터 제거 (g) ━━━
  float linX = ax - gravX;
  float linY = ay - gravY;
  float linZ = az - gravZ;
  float aNet = sqrt(linX*linX + linY*linY + linZ*linZ);   // 총 선형가속도 크기

  // ━━━ 4. 수평/수직 분해 — 중력 방향 기준 (회전 보정됨) ━━━
  //  vert  : 중력축 성분 (위아래),  horiz : 지면 평면 성분 (교통사고 등 x,y 충격)
  float vert = linX*gHx + linY*gHy + linZ*gHz;
  float hX = linX - vert*gHx;
  float hY = linY - vert*gHy;
  float hZ = linZ - vert*gHz;
  float aHoriz = sqrt(hX*hX + hY*hY + hZ*hZ);

  // ━━━ 5. 충격력 — 수평 성분 기준 F = m · a (N) ━━━
  float force = HEAD_MASS * aHoriz * G;

  // ━━━ 6. 충격량 J = F · Δt (N·s) ━━━
  float impulse = force * dt;

  // ━━━ 7. 저크 (g/s) — 수평 선형가속도 변화율 ━━━
  float jerk = 0;
  if (!firstFrame) {
    float dHx = hX - prevHx;
    float dHy = hY - prevHy;
    float dHz = hZ - prevHz;
    jerk = sqrt(dHx*dHx + dHy*dHy + dHz*dHz) / dt;
  }

  // ━━━ 8. 자세각 (°) — 추정 중력 벡터 기준 (회전 안정적) ━━━
  float roll  = atan2(gravY, gravZ) * 180.0 / M_PI;
  float pitch = atan2(-gravX, sqrt(gravY*gravY + gravZ*gravZ)) * 180.0 / M_PI;

  // ━━━ 9. 각속도 크기 (°/s) ━━━
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  // ━━━ 피크 갱신 ━━━
  if (aMag > peakG)     peakG = aMag;
  if (force > peakForce) peakForce = force;

  // ━━━ 충격 판정 — 수평(x,y) 선형가속도 기준 ━━━
  bool impact = (aHoriz >= IMPACT_THRESH);

  // ━━━━━━ 출력 (CSV, TUI에서 파싱) ━━━━━━
  // 형식: IMU,ax,ay,az,gx,gy,gz,temp,light,fsr,alcohol,aMag,aNet,aHoriz,force,impulse,jerk,roll,pitch,gyroMag,peakG,peakForce,impact
  Serial.print(F("IMU,"));
  Serial.print(ax, 3);       Serial.print(',');
  Serial.print(ay, 3);       Serial.print(',');
  Serial.print(az, 3);       Serial.print(',');
  Serial.print(gx, 2);       Serial.print(',');
  Serial.print(gy, 2);       Serial.print(',');
  Serial.print(gz, 2);       Serial.print(',');
  Serial.print(temp, 1);     Serial.print(',');
  Serial.print(light);       Serial.print(',');
  Serial.print(fsr);         Serial.print(',');
  Serial.print(alcohol);     Serial.print(',');
  Serial.print(aMag, 3);     Serial.print(',');
  Serial.print(aNet, 3);     Serial.print(',');
  Serial.print(aHoriz, 3);   Serial.print(',');
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
  prevHx = hX; prevHy = hY; prevHz = hZ;
  firstFrame = false;
}
