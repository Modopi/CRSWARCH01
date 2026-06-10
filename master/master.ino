#include <SoftwareSerial.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

//zs 040
SoftwareSerial hc06(2, 3);

// ── 2× I2C 16x2 LCD (드라이버 백팩) ──
//  같은 I2C 버스(A4=SDA, A5=SCL) 공유 → 주소가 서로 달라야 함.
//  I2C 스캐너로 실제 주소 확인 후 아래 값 맞추기.
#define LCD_LABEL_ADDR 0x27       // 라벨 LCD (버튼 기능 표시)
#define LCD_INFO_ADDR  0x26       // 정보 LCD (라이브 값) — A0 점퍼 단락으로 0x27→0x26
LiquidCrystal_I2C lcdLabel(LCD_LABEL_ADDR, 16, 2);
LiquidCrystal_I2C lcdInfo (LCD_INFO_ADDR,  16, 2);

// ── 4x4 키패드 (상단 8키만 사용, 나머지는 가림) ──
//  Row=D5~D8, Col=A0~A3(디지털). BT(D2,D3)/I2C(A4,A5) 핀과 충돌 없음.
const byte KP_ROWS = 4, KP_COLS = 4;
char keymap[KP_ROWS][KP_COLS] = {
  {'1','2','3','A'},   // 1,2,3 사용 / A 가림
  {'4','5','6','B'},   // 4,5,6 사용 / B 가림
  {'7','8','9','C'},   // 7,8 사용 / 9,C 가림
  {'*','0','#','D'},   // 전부 가림 (오버레이로 1~8만 노출)
};
byte rowPins[KP_ROWS] = {5, 6, 7, 8};
byte colPins[KP_COLS] = {A0, A1, A2, A3};
Keypad keypad = Keypad(makeKeymap(keymap), rowPins, colPins, KP_ROWS, KP_COLS);

// ── 실제 사용하는 8키 (배선: col3·col4, S4가 맨 위) ──
//  세로 분리 배치: 기능1~4 = col3(S3,S7,S11,S15), 기능5~8 = col4(S4,S8,S12,S16).
//  오버레이에 1~8로 라벨링 → LCD 범례의 숫자와 일치.
#define BTN1 '3'   // col3 row1 (S3)
#define BTN2 '6'   // col3 row2 (S7)
#define BTN3 '9'   // col3 row3 (S11)
#define BTN4 '#'   // col3 row4 (S15)
#define BTN5 'A'   // col4 row1 (S4, 맨 위)
#define BTN6 'B'   // col4 row2 (S8)
#define BTN7 'C'   // col4 row3 (S12)
#define BTN8 'D'   // col4 row4 (S16)

// ── UI 상태 ──
//  화면 모드: HOME(릴레이 제어) ↔ SENSOR(센서 상세). 키패드 기능이 모드별로 달라짐.
enum UiMode { MODE_HOME = 0, MODE_SENSOR = 1 };
uint8_t uiMode    = MODE_HOME;
uint8_t relayMode = 2;            // master가 명령하는 모드 (0=OFF,1=ON,2=AUTO)
uint8_t sensorSel = 0;            // SENSOR 모드 선택: 0=조도 1=알콜 2=압력 3=가속도
uint8_t scrollRow = 0;            // SENSOR 상세 스크롤 시작 줄
bool    freeze    = false;        // 화면 정지(Hold) — 값 갱신 멈춤
bool    lcdDirty  = true;
unsigned long lastHeartbeat = 0;
unsigned long lastLcdRefresh = 0;
const unsigned long HEARTBEAT_MS  = 1000;  // 릴레이 모드 재전송 주기
const unsigned long LCD_REFRESH_MS = 250;  // 정보 LCD 갱신 상한 (4Hz, I2C 느림)

// ── LCD 표시용 최신값 ──
float dispTemp = 0, dispPeakG = 0, dispPeakF = 0;
float dispAMag = 0, dispAHoriz = 0, dispForce = 0, dispRoll = 0, dispPitch = 0;
int   dispLight = 0, dispFsr = 0, dispAlcohol = 0;
uint8_t slaveRelayMode = 2, slaveRelayState = 0;   // slave가 보고한 실제 상태

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

// ── 릴레이 모드 명령 송신 (R,<mode>\n) ──
void sendRelayCmd() {
  hc06.print('R');
  hc06.print(',');
  hc06.println(relayMode);
}

// LCD 한 줄을 16칸으로 패딩해 출력 (이전 잔상 제거)
void lcdLine(LiquidCrystal_I2C &lcd, uint8_t row, const char *s) {
  char buf[17];
  uint8_t i = 0;
  for (; i < 16 && s[i]; i++) buf[i] = s[i];
  for (; i < 16; i++) buf[i] = ' ';
  buf[16] = '\0';
  lcd.setCursor(0, row);
  lcd.print(buf);
}

// 라벨 LCD — 현재 모드에 맞는 버튼 범례 출력 (모드/Hold 바뀔 때만 호출)
void setLabelForMode() {
  if (uiMode == MODE_SENSOR) {
    lcdLine(lcdLabel, 0, "1LT 2AL 3PR 4AC");
    lcdLine(lcdLabel, 1, freeze ? "5Exit 6HELD 7v8^" : "5Exit 6Hold 7v8^");
  } else {
    lcdLine(lcdLabel, 0, "1AUTO 2OFF 3ON");
    lcdLine(lcdLabel, 1, "4:SENSOR MODE");
  }
}

// SENSOR 상세 줄 목록 생성 (선택 센서별, 라이브 값). 줄 수 반환.
uint8_t buildSensorLines(char lines[][17]) {
  char a[12];
  uint8_t n = 0;
  switch (sensorSel) {
    case 0:  // 조도 (CDS)
      snprintf(lines[n++], 17, "Light  raw%4d", dispLight);
      dtostrf(dispLight * 100.0 / 1023.0, 1, 1, a);
      snprintf(lines[n++], 17, "pct  %s %%", a);
      snprintf(lines[n++], 17, "th <550=>ON");
      snprintf(lines[n++], 17, "now  %s", dispLight < 550 ? "DARK" : "BRIGHT");
      break;
    case 1:  // 알콜 (MQ-3)
      snprintf(lines[n++], 17, "Alcohol raw%4d", dispAlcohol);
      dtostrf(dispAlcohol * 100.0 / 1023.0, 1, 1, a);
      snprintf(lines[n++], 17, "pct  %s %%", a);
      snprintf(lines[n++], 17, "th >500 DRUNK");
      snprintf(lines[n++], 17, "now  %s", dispAlcohol >= 500 ? "DRUNK" : "OK");
      break;
    case 2:  // 압력 (FSR402)
      snprintf(lines[n++], 17, "Press  raw%4d", dispFsr);
      dtostrf(dispFsr * 100.0 / 1023.0, 1, 1, a);
      snprintf(lines[n++], 17, "pct  %s %%", a);
      snprintf(lines[n++], 17, "state %s",
               dispFsr < 100 ? "NONE" : dispFsr < 500 ? "LIGHT" : "FIRM");
      break;
    default: // 가속도 (MPU-6050) — 현재값 + 피크
      dtostrf(dispAMag, 1, 2, a);   snprintf(lines[n++], 17, "aMag  %s g", a);
      dtostrf(dispAHoriz, 1, 2, a); snprintf(lines[n++], 17, "aHoriz %s g", a);
      dtostrf(dispForce, 1, 0, a);  snprintf(lines[n++], 17, "Force %s N", a);   // 현재 충격력
      dtostrf(dispPeakG, 1, 2, a);  snprintf(lines[n++], 17, "peakG %s g", a);   // 피크
      dtostrf(dispPeakF, 1, 0, a);  snprintf(lines[n++], 17, "peakF %s N", a);   // 피크 충격력
      dtostrf(dispRoll, 1, 0, a);   snprintf(lines[n++], 17, "roll  %s deg", a);
      dtostrf(dispPitch, 1, 0, a);  snprintf(lines[n++], 17, "pitch %s deg", a);
      break;
  }
  return n;
}

// 정보 LCD 갱신 — 모드별 분기
void updateInfoLcd() {
  if (uiMode == MODE_SENSOR) {
    char lines[8][17];
    uint8_t n = buildSensorLines(lines);
    if (scrollRow >= n) scrollRow = n - 1;            // 스크롤 범위 클램프
    lcdLine(lcdInfo, 0, lines[scrollRow]);
    lcdLine(lcdInfo, 1, (scrollRow + 1 < n) ? lines[scrollRow + 1] : "");
    return;
  }
  // HOME: 0행 릴레이 상태, 1행 피크 + 센서모드 안내
  char l0[20], l1[20], a[10];
  const char *mn = (slaveRelayMode == 2) ? "AUTO" : (slaveRelayMode == 1) ? "ON  " : "OFF ";
  const char *rs = slaveRelayState ? "ON " : "OFF";
  snprintf(l0, sizeof(l0), "Relay %s [%s]", mn, rs);
  dtostrf(dispPeakG, 1, 1, a);
  snprintf(l1, sizeof(l1), "Pk %sg   4=SENS", a);
  lcdLine(lcdInfo, 0, l0);
  lcdLine(lcdInfo, 1, l1);
}

// 키패드 입력 — 현재 모드에 따라 의미가 달라짐 (사용 키 BTN1~BTN8)
void handleKey(char k) {
  if (uiMode == MODE_SENSOR) {
    switch (k) {
      case BTN1: sensorSel = 0; scrollRow = 0; lcdDirty = true; break;  // 조도
      case BTN2: sensorSel = 1; scrollRow = 0; lcdDirty = true; break;  // 알콜
      case BTN3: sensorSel = 2; scrollRow = 0; lcdDirty = true; break;  // 압력
      case BTN4: sensorSel = 3; scrollRow = 0; lcdDirty = true; break;  // 가속도
      case BTN5: uiMode = MODE_HOME; freeze = false;                    // 모드 종료
                 setLabelForMode(); lcdDirty = true; break;
      case BTN6: freeze = !freeze; setLabelForMode(); lcdDirty = true; break;  // Hold
      case BTN7: scrollRow++;  lcdDirty = true; break;                  // 다음 줄
      case BTN8: if (scrollRow > 0) scrollRow--; lcdDirty = true; break;// 이전 줄
      default:   break;
    }
    return;
  }
  // HOME
  switch (k) {
    case BTN1: relayMode = 2; sendRelayCmd(); lcdDirty = true; break;  // AUTO
    case BTN2: relayMode = 0; sendRelayCmd(); lcdDirty = true; break;  // OFF
    case BTN3: relayMode = 1; sendRelayCmd(); lcdDirty = true; break;  // ON
    case BTN4: uiMode = MODE_SENSOR; sensorSel = 0; scrollRow = 0;     // 센서모드 진입
               setLabelForMode(); lcdDirty = true; break;
    default:   break;
  }
}

void setup() {
  Serial.begin(115200);   // USB → PC(TUI). 하드웨어 UART라 115200 안정 (긴 줄 블로킹 최소화)
  hc06.begin(38400);      // BT ← slave. SoftwareSerial 안정 상한 = 38400
  prevTime = millis();

  // LCD 초기화 — 라벨 LCD는 현재 모드의 버튼 범례 표시
  Wire.begin();
  lcdLabel.init(); lcdLabel.backlight();
  lcdInfo.init();  lcdInfo.backlight();
  setLabelForMode();      // HOME 범례
  lcdLine(lcdInfo,  0, "Relay AUTO [OFF]");
  lcdLine(lcdInfo,  1, "  waiting BT...");
  sendRelayCmd();         // 초기 모드(AUTO) 동기화

  // 헤더 — TUI는 "IMU,"로 시작하지 않는 줄을 무시함
  Serial.println(F("# Smart Helmet Impact Monitor — CSV stream"));
  Serial.println(F("# fields: IMU,ax,ay,az,gx,gy,gz,temp,light,fsr,alcohol,aMag,aNet,aHoriz,force,impulse,jerk,roll,pitch,gyroMag,peakG,peakForce,impact,relayMode,relayState"));
  Serial.println(F("# event:  IMPACT,peakG,peakForce,impulse,durMs,peakJerk  (slave 고속 적분 결과)"));
}

void loop() {
  // ── 키패드 입력 ──
  char k = keypad.getKey();
  if (k) handleKey(k);

  unsigned long uiNow = millis();

  // ── 릴레이 모드 하트비트 (1s) — 드롭된 명령 자동 복구 ──
  if (uiNow - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = uiNow;
    sendRelayCmd();
  }

  // ── 정보 LCD 갱신 (4Hz 상한, I2C가 느려 BT 스트림 블로킹 최소화) ──
  if (lcdDirty && uiNow - lastLcdRefresh >= LCD_REFRESH_MS) {
    lastLcdRefresh = uiNow;
    lcdDirty = false;
    updateInfoLcd();
  }

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

  // ax, ay, az, gx, gy, gz, temp, light, fsr, alcohol, relayMode, relayState
  float v[12];
  if (!parseCSV(line, v, 12)) return;

  float ax = v[0], ay = v[1], az = v[2];   // g
  float gx = v[3], gy = v[4], gz = v[5];   // °/s
  float temp = v[6];                         // °C
  int   light   = (int)v[7];                 // 0..1023 (raw ADC)
  int   fsr     = (int)v[8];                 // 0..1023 (raw ADC, FSR402)
  int   alcohol = (int)v[9];                 // 0..1023 (raw ADC, MQ-3)
  int   relayModeRpt  = (int)v[10];          // slave가 적용 중인 모드 (0/1/2)
  int   relayStateRpt = (int)v[11];          // slave 실제 릴레이 출력 (0/1)

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

  // ━━━ LCD 표시값 갱신 (실제 쓰기는 loop의 4Hz throttle에서) ━━━
  //  Hold(freeze)면 값 갱신을 멈춰 화면을 고정 — 단 릴레이 상태는 항상 추종.
  slaveRelayMode = relayModeRpt; slaveRelayState = relayStateRpt;
  if (!freeze) {
    dispTemp = temp; dispLight = light; dispFsr = fsr; dispAlcohol = alcohol;
    dispPeakG = peakG; dispPeakF = peakForce;
    dispAMag = aMag; dispAHoriz = aHoriz; dispForce = force; dispRoll = roll; dispPitch = pitch;
    lcdDirty = true;
  }

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
  Serial.print(impact ? 1 : 0); Serial.print(',');
  Serial.print(relayModeRpt);  Serial.print(',');
  Serial.println(relayStateRpt);

  // ── 다음 프레임 준비 ──
  prevHx = hX; prevHy = hY; prevHz = hZ;
  firstFrame = false;
}
