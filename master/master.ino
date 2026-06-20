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
#define BUZZER_PIN     4          // 패시브 부저 (D4)
LiquidCrystal_I2C lcdLabel(LCD_LABEL_ADDR, 16, 2);
LiquidCrystal_I2C lcdInfo (LCD_INFO_ADDR,  16, 2);

// ── IR 장애물 센서 2개 (D9, D11) — 손잡이 파지 여부를 디지털 0/1로만 판정 ──
//  모듈 OUT: 손(장애물) 감지 시 LOW, 손을 떼면 HIGH. (모듈 극성이 반대면 pollLocalSensors의 비교를 뒤집기)
//  판정: 둘 중 하나라도 감지(OR)되면 파지 → "둘 다 떼야" 경고.
#define IR_HANDLE_PIN1 9
#define IR_HANDLE_PIN2 11
const unsigned long HANDLE_GRACE_MS = 1500;   // 손 뗀 뒤 경고까지 유예(신호용 잠깐 떼기 무시)
bool          gripA = false, gripB = false;   // 각 센서 파지 감지(true=손 있음)
bool          handleHeld     = true;          // 손잡이 파지 중?(둘 중 하나라도 감지)
bool          prevHandleHeld = true;          // 직전 상태(변화 감지용)
unsigned long handleOffSince = 0;             // 손을 (둘 다) 뗀 시각

// ── DS18B20 디지털 온도 센서 (D10, 1-Wire) — 배터리 온도 정밀 감시 (자체 리더, 라이브러리 미사용) ──
#define DS18_PIN 10
const int           BATT_TEMP_WARN = 45;      // 배터리 과열 경고 임계 (°C)
const unsigned long DS_READ_MS     = 1500;    // 폴링 주기(변환 750ms보다 길게)
int     battTemp = 0;                         // 최근 온도 정수부 (°C)
uint8_t battFrac = 0;                         // 소수 첫째자리 (0.1°C)
bool    battValid    = false;                 // 마지막 읽기 성공 여부
bool    dsRequested  = false;                 // 직전 주기에 변환을 시작했는지
unsigned long lastDsRead = 0;

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
enum UiMode { MODE_HOME = 0, MODE_SENSOR = 1, MODE_STATUS = 2 };
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

// ── BT 수신 버퍼 (String 미사용 — 힙 단편화/블로킹 제거) ──
//  비차단 라인 리더. \n까지 모으되, 이상 문자/과길이는 즉시 폐기하고 다음 \n에서 재동기.
char    rxBuf[96];
uint8_t rxLen  = 0;
bool    rxDrop = false;   // 이 라인은 깨졌으니 버린다 (다음 \n까지)

// ── CSV 파싱 (char 버퍼, 비파괴) ──
//  정확히 count개 필드일 때만 true. 필드 수 초과/부족(병합·절단 라인) 모두 거부.
//  atof는 각 필드 시작부터 ',' 직전까지만 읽으므로 구분자에서 자동으로 멈춤.
bool parseCSV(const char *line, float *vals, int count) {
  int idx = 0;
  const char *start = line;
  for (const char *p = line; ; p++) {
    if (*p == ',' || *p == '\0') {
      if (idx >= count) return false;        // 필드 초과 → 거부
      vals[idx++] = atof(start);
      start = p + 1;
      if (*p == '\0') break;
    }
  }
  return idx == count;                        // 정확히 count개일 때만 통과
}

// ── IMU 라인 물리 범위 검증 (센서 한계 밖 = 깨진 패킷) ──
//  atof가 쓰레기 입력에서도 그럴듯한 숫자를 내므로, 값 범위로 한 번 더 거른다.
bool imuSane(const float *v) {
  for (int i = 0; i < 3; i++) if (fabs(v[i]) > 17.0)   return false;  // accel ±16g
  for (int i = 3; i < 6; i++) if (fabs(v[i]) > 2100.0) return false;  // gyro ±2000°/s
  if (v[6] < -50.0 || v[6] > 150.0) return false;                     // temp °C
  if (v[7] < 0 || v[7] > 1023) return false;                          // light
  if (v[8] < 0 || v[8] > 1023) return false;                          // fsr
  if (v[9] < 0 || v[9] > 1023) return false;                          // alcohol
  if (v[10] < 0 || v[10] > 2)  return false;                          // relayMode
  if (v[11] < 0 || v[11] > 1)  return false;                          // relayState
  return true;
}

// ── 패시브 부저 — 비차단 멜로디 플레이어 ──
// 멜로디: {주파수,지속ms, ...} 쌍 배열, 주파수 0 = 묵음
static const uint16_t MEL_IMPACT_MILD[]   = {880,150, 0,60, 1320,200};
static const uint16_t MEL_IMPACT_SEVERE[] = {1200,300, 0,80, 1200,300, 0,80, 1200,300,
                                              0,80, 1200,300, 0,80, 1200,300};
static const uint16_t MEL_DRUNK[]         = {380,500, 0,500};
static const uint16_t MEL_NOWEAR[]        = {2200,100, 0,100};
static const uint16_t MEL_HANDOFF[]       = {1500,90, 0,500};                 // 손잡이 놓침 — 느린 단음
static const uint16_t MEL_BATTHOT[]       = {2000,180, 0,120, 2400,180, 0,700}; // 배터리 과열 — 상승 2음

#define MEL_NOTES(m) (uint8_t)(sizeof(m)/sizeof(m[0])/2)
// 우선순위: 숫자 큰 쪽이 낮은 쪽을 선점 (충격 > 과열 > 음주 > 미착용 > 손잡이)
#define BZR_IDLE    0
#define BZR_HANDOFF 1
#define BZR_NOWEAR  2
#define BZR_DRUNK   3
#define BZR_BATTHOT 4
#define BZR_MILD    5
#define BZR_SEVERE  6

uint8_t         bzrActive  = BZR_IDLE;
const uint16_t *bzrMelody  = nullptr;
uint8_t         bzrNotes   = 0;
uint8_t         bzrIdx     = 0;
unsigned long   bzrNoteEnd = 0;
bool            bzrLoop    = false;

static void _bzrNote() {
  uint16_t freq = bzrMelody[bzrIdx * 2];
  uint16_t dur  = bzrMelody[bzrIdx * 2 + 1];
  if (freq) tone(BUZZER_PIN, freq, dur);
  else      noTone(BUZZER_PIN);
  bzrNoteEnd = millis() + dur;
}

static void _buzzPlay(uint8_t prio, const uint16_t *mel, uint8_t notes, bool loop) {
  if (prio <= bzrActive) return;
  bzrActive = prio; bzrMelody = mel; bzrNotes = notes; bzrIdx = 0; bzrLoop = loop;
  _bzrNote();
}

void buzzUpdate() {
  if (bzrActive == BZR_IDLE || millis() < bzrNoteEnd) return;
  bzrIdx++;
  if (bzrIdx >= bzrNotes) {
    if (!bzrLoop) { noTone(BUZZER_PIN); bzrActive = BZR_IDLE; return; }
    bzrIdx = 0;
  }
  _bzrNote();
}

void buzzStop(uint8_t prio) {
  if (bzrActive == prio) { noTone(BUZZER_PIN); bzrActive = BZR_IDLE; }
}

void buzzImpactMild()    { _buzzPlay(BZR_MILD,   MEL_IMPACT_MILD,   MEL_NOTES(MEL_IMPACT_MILD),   false); }
void buzzImpactSevere()  { _buzzPlay(BZR_SEVERE,  MEL_IMPACT_SEVERE, MEL_NOTES(MEL_IMPACT_SEVERE), false); }
void buzzDrunk(bool on)  { if (on) _buzzPlay(BZR_DRUNK,  MEL_DRUNK,  MEL_NOTES(MEL_DRUNK),  true); else buzzStop(BZR_DRUNK); }
void buzzNoWear(bool on) { if (on) _buzzPlay(BZR_NOWEAR, MEL_NOWEAR, MEL_NOTES(MEL_NOWEAR), true); else buzzStop(BZR_NOWEAR); }
void buzzHandOff(bool on){ if (on) _buzzPlay(BZR_HANDOFF, MEL_HANDOFF, MEL_NOTES(MEL_HANDOFF), true); else buzzStop(BZR_HANDOFF); }
void buzzBattHot(bool on){ if (on) _buzzPlay(BZR_BATTHOT, MEL_BATTHOT, MEL_NOTES(MEL_BATTHOT), true); else buzzStop(BZR_BATTHOT); }

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
  } else if (uiMode == MODE_STATUS) {
    lcdLine(lcdLabel, 0, "STATUS  (local)");
    lcdLine(lcdLabel, 1, freeze ? "5Exit 6HELD 7v8^" : "5Exit 6Hold 7v8^");
  } else {
    lcdLine(lcdLabel, 0, "1AUTO 2OFF 3ON");
    lcdLine(lcdLabel, 1, "4SENS 5STATUS");
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

// STATUS(로컬 센서) 상세 줄 — IR 손잡이 그립 + DS18B20 배터리 온도. 줄 수 반환.
//  포맷 문자열은 PSTR/snprintf_P로 PROGMEM에 둬 SRAM 절약(AVR은 일반 리터럴을 RAM에 복사).
uint8_t buildStatusLines(char lines[][17]) {
  uint8_t n = 0;
  // 손잡이 그립 (디지털 0/1)
  snprintf_P(lines[n++], 17, PSTR("Grip   %s"), handleHeld ? "HELD" : "OFF !");
  snprintf_P(lines[n++], 17, PSTR("IR a%d b%d  %s"), gripA ? 1 : 0, gripB ? 1 : 0, handleHeld ? "hold" : "warn");
  // 배터리 온도 (DS18B20)
  if (battValid) {
    snprintf_P(lines[n++], 17, PSTR("Bat T %d.%d C"), battTemp, battFrac);
    snprintf_P(lines[n++], 17, PSTR("th >=%dC HOT"), BATT_TEMP_WARN);
    snprintf_P(lines[n++], 17, PSTR("now    %s"), battTemp >= BATT_TEMP_WARN ? "HOT!" : "OK");
  } else {
    snprintf_P(lines[n++], 17, PSTR("Bat T --.- C"));
    snprintf_P(lines[n++], 17, PSTR("DS18B20 no rd"));
  }
  return n;
}

// 정보 LCD 갱신 — 모드별 분기
void updateInfoLcd() {
  if (uiMode == MODE_STATUS) {
    char lines[8][17];
    uint8_t n = buildStatusLines(lines);
    if (scrollRow >= n) scrollRow = n - 1;            // 스크롤 범위 클램프
    lcdLine(lcdInfo, 0, lines[scrollRow]);
    lcdLine(lcdInfo, 1, (scrollRow + 1 < n) ? lines[scrollRow + 1] : "");
    return;
  }
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
  if (uiMode == MODE_STATUS) {
    switch (k) {
      case BTN5: uiMode = MODE_HOME; freeze = false;                   // 모드 종료
                 setLabelForMode(); lcdDirty = true; break;
      case BTN6: freeze = !freeze; setLabelForMode(); lcdDirty = true; break;  // Hold
      case BTN7: scrollRow++;  lcdDirty = true; break;                 // 다음 줄
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
    case BTN5: uiMode = MODE_STATUS; scrollRow = 0;                    // 로컬센서(그립/배터리)
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
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IR_HANDLE_PIN1, INPUT);  // IR 손잡이 그립 센서 1 (디지털 입력)
  pinMode(IR_HANDLE_PIN2, INPUT);  // IR 손잡이 그립 센서 2
  pinMode(DS18_PIN, INPUT_PULLUP); // DS18B20 1-Wire 라인 — 내부 풀업 보조(외부 4.7k 권장)
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

// ── 검증된 한 라인 처리 (BT에서 \n으로 잘린 완성 라인) ──
void processLine(char *line) {
  // ── 충격 이벤트 (slave 고속 적분 결과) → 검증 후 PC 전달 + 부저 ──
  //  slave 형식: E,peakG,peakForce,impulse,durMs,peakJerk
  if (line[0] == 'E' && line[1] == ',') {
    float ev[5];
    if (!parseCSV(line + 2, ev, 5)) return;        // 필드 수 불일치 → 깨진 이벤트, 무시
    // ev[0]=peakG, ev[1]=peakF(N). 센서 ±16g 한계 → peakF ≤ 6·16·9.807 ≈ 941N.
    //  범위 밖이면 깨진 패킷 → 오경보(SEVERE 오작동) 방지 위해 폐기.
    if (ev[0] < 0 || ev[0] > 100.0)  return;
    if (ev[1] < 0 || ev[1] > 1000.0) return;
    Serial.print(F("IMPACT,"));
    Serial.println(line + 2);
    float eAhoriz = ev[1] / (6.0f * 9.807f);        // 역산: N → g (aHoriz)
    if (eAhoriz >= 10.0f)      buzzImpactSevere();
    else if (eAhoriz >= 5.0f)  buzzImpactMild();
    return;
  }

  // ax, ay, az, gx, gy, gz, temp, light, fsr, alcohol, relayMode, relayState
  float v[12];
  if (!parseCSV(line, v, 12)) return;   // 필드 수 불일치(병합·절단) → 폐기
  if (!imuSane(v))            return;   // 물리 범위 밖(깨진 바이트) → 폐기

  float ax = v[0], ay = v[1], az = v[2];   // g
  float gx = v[3], gy = v[4], gz = v[5];   // °/s
  float temp = v[6];                         // °C
  int   light   = (int)v[7];                 // 0..1023 (raw ADC)
  int   fsr     = (int)v[8];                 // 0..1023 (raw ADC, FSR402)
  int   alcohol = (int)v[9];                 // 0..1023 (raw ADC, MQ-3)
  int   relayModeRpt  = (int)v[10];          // slave가 적용 중인 모드 (0/1/2)
  int   relayStateRpt = (int)v[11];          // slave 실제 릴레이 출력 (0/1)

  // ── 위험 신호 → 부저 ──
  buzzDrunk(alcohol >= 500);
  buzzNoWear(fsr < 100);

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

// ── DS18B20 자체 1-Wire 리더 (라이브러리 미사용) ──
//  1-Wire는 타이밍이 전부 "고정 delayMicroseconds"라 엣지를 무한정 기다리는 루프가 없음
//  → 무한 hang 구조적으로 불가능(센서 없으면 owReset이 presence 미검출로 즉시 false).
//  µs 펄스가 타이머·BT 인터럽트에 밀리면 정확도만 떨어지므로 비트 임계구간만 noInterrupts로 보호.

// 1-Wire 한 비트 쓰기 (1: 6µs LOW; 0: 60µs LOW)
static void owWriteBit(uint8_t bit) {
  noInterrupts();
  pinMode(DS18_PIN, OUTPUT); digitalWrite(DS18_PIN, LOW);
  if (bit) { delayMicroseconds(6);  pinMode(DS18_PIN, INPUT_PULLUP); delayMicroseconds(64); }
  else     { delayMicroseconds(60); pinMode(DS18_PIN, INPUT_PULLUP); delayMicroseconds(10); }
  interrupts();
}
// 1-Wire 한 비트 읽기 (LOW 6µs 후 풀고 9µs 뒤 샘플)
static uint8_t owReadBit() {
  uint8_t b;
  noInterrupts();
  pinMode(DS18_PIN, OUTPUT); digitalWrite(DS18_PIN, LOW);
  delayMicroseconds(6);
  pinMode(DS18_PIN, INPUT_PULLUP);
  delayMicroseconds(9);
  b = digitalRead(DS18_PIN);
  interrupts();
  delayMicroseconds(55);
  return b;
}
// 리셋 펄스 + presence 검출 (장치 있으면 1). 고정 타이밍 → 절대 hang 안 함.
static uint8_t owReset() {
  uint8_t presence;
  pinMode(DS18_PIN, OUTPUT); digitalWrite(DS18_PIN, LOW);
  delayMicroseconds(480);
  noInterrupts();
  pinMode(DS18_PIN, INPUT_PULLUP);
  delayMicroseconds(70);
  presence = !digitalRead(DS18_PIN);   // 장치가 라인을 LOW로 당기면 존재
  interrupts();
  delayMicroseconds(410);
  return presence;
}
static void    owWrite(uint8_t v) { for (uint8_t i = 0; i < 8; i++) { owWriteBit(v & 1); v >>= 1; } }
static uint8_t owRead()           { uint8_t v = 0; for (uint8_t i = 0; i < 8; i++) { v >>= 1; if (owReadBit()) v |= 0x80; } return v; }

// Dallas/Maxim CRC8
static uint8_t owCRC8(const uint8_t *d, uint8_t n) {
  uint8_t crc = 0;
  while (n--) {
    uint8_t b = *d++;
    for (uint8_t i = 0; i < 8; i++) {
      uint8_t mix = (crc ^ b) & 0x01;
      crc >>= 1; if (mix) crc ^= 0x8C; b >>= 1;
    }
  }
  return crc;
}

// 온도 변환 시작 (비동기, ~750ms 뒤 완료). 장치 없으면 false.
bool ds18StartConvert() {
  if (!owReset()) return false;
  owWrite(0xCC);   // SKIP ROM (버스에 장치 1개 가정)
  owWrite(0x44);   // CONVERT T
  return true;
}
// 스크래치패드 읽어 raw 온도(1/16°C) 반환. CRC 통과 시 true.
bool ds18ReadRaw(int16_t *raw) {
  if (!owReset()) return false;
  owWrite(0xCC);   // SKIP ROM
  owWrite(0xBE);   // READ SCRATCHPAD
  uint8_t sp[9], orv = 0, andv = 0xFF;
  for (uint8_t i = 0; i < 9; i++) { sp[i] = owRead(); orv |= sp[i]; andv &= sp[i]; }
  // 풀업 없음/통신 안 됨 → 라인이 전부 0(또는 전부 1)로 읽힘. 전부-0은 CRC(0)==0으로 통과돼
  // 가짜 0.0°C가 되므로 반드시 따로 거른다. (정상 스크래치패드는 config·reserved 바이트가 non-zero)
  if (orv == 0 || andv == 0xFF)  return false;
  if (owCRC8(sp, 8) != sp[8])    return false;   // CRC 불일치 → 깨진 읽기
  *raw = (int16_t)((sp[1] << 8) | sp[0]);
  return true;
}

// ── 마스터 로컬 센서 폴링 — IR 손잡이 그립(디지털) + DS18B20 배터리 온도 ──
//  BT/부저/키패드를 막지 않도록: IR은 매 루프(digitalRead 즉시), DS18B20은 1.5s마다(교환 ~수 ms).
void pollLocalSensors() {
  unsigned long now = millis();

  // ── IR 손잡이 그립 (2개) — 손 있으면 LOW=파지, HIGH=뗌. 둘 중 하나라도 잡으면 파지 ──
  gripA = (digitalRead(IR_HANDLE_PIN1) == LOW);
  gripB = (digitalRead(IR_HANDLE_PIN2) == LOW);
  bool present = gripA || gripB;
  if (present) {
    handleHeld = true;
  } else if (handleHeld) {
    handleHeld = false; handleOffSince = now;   // 둘 다 막 뗀 순간 기록
  }

  // ── DS18B20 배터리 온도 (1.5s 주기, 비동기 변환) ──
  //  변환은 ~750ms 걸리므로: 지난 주기에 시작해 둔 변환 결과를 읽고 → 곧바로 다음 변환을 시작.
  //  덕분에 750ms 대기를 블로킹하지 않음(읽기/시작 합쳐 ~수 ms). 1-Wire는 고정 타이밍이라 hang 불가.
  //  µs 펄스 정확도를 위해 교환 동안만 BT 수신 중지(stopListening)→교환→재개(listen).
  if (now - lastDsRead >= DS_READ_MS) {
    lastDsRead = now;
    int16_t raw;
    hc06.stopListening();
    bool ok = dsRequested && ds18ReadRaw(&raw);   // 지난 주기 변환 결과 읽기
    dsRequested = ds18StartConvert();             // 다음 변환 시작(비동기)
    hc06.listen();
    if (ok) {
      battTemp = raw >> 4;                         // 정수부 (raw 1LSB = 1/16°C)
      battFrac = (uint8_t)(((raw & 0x0F) * 10) >> 4);  // 소수 첫째자리
      battValid = true;
    } else {
      battValid = false;                          // 읽기 실패/장치 없음 — 과열 경고 보류
    }
    if (uiMode == MODE_STATUS && !freeze) lcdDirty = true;
  }

  // ── 위험 신호 → 부저 (매 루프 평가: 상위음에 선점됐다 풀리면 자동 복귀) ──
  buzzHandOff(!handleHeld && (now - handleOffSince >= HANDLE_GRACE_MS));
  buzzBattHot(battValid && battTemp >= BATT_TEMP_WARN);

  // ── 그립 상태 변화 시 STATUS 화면 갱신 ──
  if (handleHeld != prevHandleHeld) {
    prevHandleHeld = handleHeld;
    if (uiMode == MODE_STATUS && !freeze) lcdDirty = true;
  }
}

void loop() {
  buzzUpdate();
  pollLocalSensors();

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

  // ── BT 수신 — 비차단 라인 리더 (String 미사용: 힙 단편화/블로킹 제거) ──
  //  available()인 동안만 읽어 한 바이트도 블로킹하지 않음 → 부저 시퀀싱·키패드 응답 유지.
  //  \r·\n에서 라인 확정. 프로토콜 외 문자나 과길이 라인은 rxDrop으로 통째 폐기하고
  //  다음 줄바꿈에서 자동 재동기 → tone()/BT 잡음으로 깨진 바이트가 값에 새지 않음.
  while (hc06.available()) {
    char c = hc06.read();
    if (c == '\n' || c == '\r') {
      if (!rxDrop && rxLen > 0) { rxBuf[rxLen] = '\0'; processLine(rxBuf); }
      rxLen = 0; rxDrop = false;
    } else if (rxDrop) {
      // 이미 폐기 중 — 다음 줄바꿈까지 무시
    } else if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == ',' || c == 'E') {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
      else rxDrop = true;                 // 과길이(라인 병합) → 폐기
    } else {
      rxDrop = true;                       // 프로토콜 외 문자(깨진 바이트) → 폐기
    }
  }
}
