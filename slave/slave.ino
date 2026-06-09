#include <Wire.h>
#include <SoftwareSerial.h>

//정품보드(빨강+파랑불)이 슬레이브
SoftwareSerial hc06(2, 3);  // RX, TX

const int RELAY_PIN = 7;        // 릴레이 출력
const int LIGHT_THRESHOLD = 550; // 이 값 미만이면 릴레이 ON


const uint8_t MPU_ADDR = 0x68;
const uint8_t LIGHT_PIN = A0;   // 조도 센서 (CDS)
const uint8_t FSR_PIN   = A1;   // FSR402 압력 센서 (풀다운)
const uint8_t ALCOHOL_PIN = A2; // MQ-3 알콜 센서 (AO)

// ── 충격 물리 파라미터 (master와 동일 정의) ──
const float HEAD_MASS     = 6.0;    // 머리+헬멧 추정 질량 (kg)
const float G             = 9.807;  // m/s²
const float IMPACT_THRESH = 2.0;    // 충격 판정 — 수평 선형가속도 (g)
const float COMP_ALPHA    = 0.98;   // 상보필터: 자이로 예측 신뢰 비중
const float DEG2RAD       = 0.0174532925f;

// ── 샘플링 (FIFO가 하드웨어로 보장) ──
//  SMPLRT_DIV=1, DLPF 1kHz base → 1000/(1+1) = 500Hz. dt가 정확히 2ms로 일정.
const float SAMPLE_DT = 0.002f;             // 500Hz → 2ms
const uint8_t  FRAME_SIZE = 12;             // FIFO 한 프레임: accel(6)+gyro(6)
const uint16_t FIFO_HIGH  = 1008;           // 이만큼 차면 오버플로 직전 → 리셋

// ── 송신 주기 (상태 스트림) ──
//  BT는 38400으로 올렸지만 그래도 전송은 ~10Hz로만. 센싱은 FIFO가 500Hz 보장.
const unsigned long TX_INTERVAL_MS = 100;   // 상태 스트림 주기 (10Hz)
unsigned long lastTx = 0;

// ── 충격 이벤트 종료 판정 (샘플 수 기준) ──
//  수평가속도가 임계 미만으로 EVENT_END_SAMPLES(=60ms) 이상 유지되면 한 충격 종료.
const uint16_t EVENT_END_SAMPLES = 30;      // 30샘플 × 2ms = 60ms

// ── 상태(라이브 표시)용 ──
float peakMagSq = 0;
float peakAx = 0, peakAy = 0, peakAz = 0;
float lastGx = 0, lastGy = 0, lastGz = 0;   // 상태 스트림용 최신 자이로
unsigned long sampleCount = 0;              // 창 동안 처리한 샘플 수 (실측 Hz 확인용)

// ── 중력벡터 추정 (상보필터) ──
float gravX = 0, gravY = 0, gravZ = 1.0;
bool  firstFrame = true;
float prevHx = 0, prevHy = 0, prevHz = 0;   // 이전 수평 선형가속도 (jerk용)

// ── 충격 이벤트 적분 상태 ──
bool  inEvent = false;
float evImpulse = 0;      // ∫|F_horiz| dt  (N·s) — 진짜 충격량
float evPeakG = 0;        // 이벤트 중 최대 |a| (g)
float evPeakF = 0;        // 이벤트 중 최대 수평 충격력 (N)
float evPeakJerk = 0;     // 이벤트 중 최대 저크 (g/s)
uint16_t evSamples = 0;   // 이벤트 시작 후 샘플 수
uint16_t evLastAbove = 0; // 마지막으로 임계 넘은 샘플 인덱스 (지속시간 계산)
uint16_t belowCount = 0;  // 연속으로 임계 아래인 샘플 수

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

// FIFO에 쌓인 바이트 수
uint16_t fifoCount() {
  uint8_t b[2];
  mpuRead(0x72, b, 2);            // FIFO_COUNT_H/L
  return ((uint16_t)b[0] << 8) | b[1];
}

// FIFO 리셋 — FIFO_RESET은 FIFO_EN=0일 때만 동작하므로 3단계로.
void fifoReset() {
  mpuWrite(0x6A, 0x00);          // USER_CTRL: FIFO 비활성
  mpuWrite(0x6A, 0x04);          // FIFO_RESET
  mpuWrite(0x6A, 0x40);          // FIFO_EN 재활성
}

void setup() {
  Serial.begin(9600);     // USB 디버그 (slave는 평소 PC에 안 붙음)
  hc06.begin(38400);      // BT → master. SoftwareSerial 안정 상한 = 38400
  Wire.begin();
  Wire.setClock(400000);  // I2C 400kHz(고속)

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // MPU-6050 초기화
  mpuWrite(0x6B, 0x00);   // 슬립 해제
  delay(100);
  mpuWrite(0x19, 0x01);   // SMPLRT_DIV=1 → 500Hz
  mpuWrite(0x1A, 0x01);   // DLPF 184Hz(가속도) — base 1kHz, 충격 스파이크 보존하면서 안정
  mpuWrite(0x1C, 0x18);   // 가속도 ±16g
  mpuWrite(0x1B, 0x18);   // 자이로 ±2000°/s

  // FIFO 설정: 가속도 + 자이로를 하드웨어 버퍼에 자동 적재
  mpuWrite(0x6A, 0x00);   // 설정 중 FIFO 비활성
  mpuWrite(0x23, 0x78);   // FIFO_EN: ACCEL(0x08) + GYRO X/Y/Z(0x70)
  fifoReset();            // 버퍼 비우고 활성화

  Serial.println("Slave ready (FIFO 500Hz)");
}

// FIFO 한 샘플 처리 — 충격 물리 계산 + 피크홀드 + 이벤트 적분 (dt 일정 = 2ms)
void processSample(float ax, float ay, float az, float gx, float gy, float gz) {
  float aMag = sqrt(ax*ax + ay*ay + az*az);

  // 중력 추정 (상보필터, dt 일정)
  if (firstFrame) {
    gravX = ax; gravY = ay; gravZ = az;
  } else {
    float wx = gx*DEG2RAD, wy = gy*DEG2RAD, wz = gz*DEG2RAD;
    float gxp = gravX - (wy*gravZ - wz*gravY) * SAMPLE_DT;
    float gyp = gravY - (wz*gravX - wx*gravZ) * SAMPLE_DT;
    float gzp = gravZ - (wx*gravY - wy*gravX) * SAMPLE_DT;
    gravX = COMP_ALPHA*gxp + (1.0f-COMP_ALPHA)*ax;
    gravY = COMP_ALPHA*gyp + (1.0f-COMP_ALPHA)*ay;
    gravZ = COMP_ALPHA*gzp + (1.0f-COMP_ALPHA)*az;
  }
  float gN = sqrt(gravX*gravX + gravY*gravY + gravZ*gravZ);
  if (gN < 1e-3) gN = 1.0;
  float gHx = gravX/gN, gHy = gravY/gN, gHz = gravZ/gN;

  // 수평 선형가속도 → 충격력
  float linX = ax - gravX, linY = ay - gravY, linZ = az - gravZ;
  float vert = linX*gHx + linY*gHy + linZ*gHz;
  float hX = linX - vert*gHx, hY = linY - vert*gHy, hZ = linZ - vert*gHz;
  float aHoriz = sqrt(hX*hX + hY*hY + hZ*hZ);   // g
  float force  = HEAD_MASS * aHoriz * G;        // N

  // 저크
  float jerk = 0;
  if (!firstFrame) {
    float dX = hX - prevHx, dY = hY - prevHy, dZ = hZ - prevHz;
    jerk = sqrt(dX*dX + dY*dY + dZ*dZ) / SAMPLE_DT;
  }
  prevHx = hX; prevHy = hY; prevHz = hZ;
  firstFrame = false;

  // 상태 표시용 피크홀드
  float magSq = ax*ax + ay*ay + az*az;
  if (magSq > peakMagSq) { peakMagSq = magSq; peakAx = ax; peakAy = ay; peakAz = az; }
  lastGx = gx; lastGy = gy; lastGz = gz;
  sampleCount++;

  // ── 충격 이벤트 상태머신 (샘플 단위) ──
  bool above = (aHoriz >= IMPACT_THRESH);
  if (above && !inEvent) {              // 새 충격 시작
    inEvent = true;
    evImpulse = 0; evPeakG = 0; evPeakF = 0; evPeakJerk = 0;
    evSamples = 0; evLastAbove = 0; belowCount = 0;
  }
  if (inEvent) {
    evImpulse += force * SAMPLE_DT;     // ∫F dt — 충격량 누적
    if (aMag > evPeakG)  evPeakG = aMag;
    if (force > evPeakF) evPeakF = force;
    if (jerk > evPeakJerk) evPeakJerk = jerk;
    evSamples++;
    if (above) { belowCount = 0; evLastAbove = evSamples; }
    else if (++belowCount >= EVENT_END_SAMPLES) {   // 충격 종료 → 결과만 전송
      unsigned long durMs = (unsigned long)(evLastAbove * SAMPLE_DT * 1000.0 + 0.5);
      char e[72], t[16];
      strcpy(e, "E,");
      dtostrf(evPeakG, 1, 3, t);    strcat(e, t); strcat(e, ",");
      dtostrf(evPeakF, 1, 1, t);    strcat(e, t); strcat(e, ",");
      dtostrf(evImpulse, 1, 4, t);  strcat(e, t); strcat(e, ",");
      ultoa(durMs, t, 10);          strcat(e, t); strcat(e, ",");
      dtostrf(evPeakJerk, 1, 1, t); strcat(e, t);
      hc06.println(e);
      Serial.println(e);
      inEvent = false;
    }
  }
}

void loop() {
  // ━━━ FIFO 드레인 — 쌓인 샘플 전부 처리 (전송 블랙아웃 동안 것도 회수) ━━━
  uint16_t count = fifoCount();
  if (count >= FIFO_HIGH) {       // 오버플로 직전 → 리셋 (정렬 깨짐 방지)
    fifoReset();
    Serial.println(F("# FIFO overflow -> reset"));
    count = 0;
  }
  while (count >= FRAME_SIZE) {
    uint8_t f[FRAME_SIZE];
    mpuRead(0x74, f, FRAME_SIZE); // FIFO_R_W에서 한 프레임
    count -= FRAME_SIZE;
    float ax = combine(f[0],  f[1])  / 2048.0;
    float ay = combine(f[2],  f[3])  / 2048.0;
    float az = combine(f[4],  f[5])  / 2048.0;
    float gx = combine(f[6],  f[7])  / 16.4;
    float gy = combine(f[8],  f[9])  / 16.4;
    float gz = combine(f[10], f[11]) / 16.4;
    processSample(ax, ay, az, gx, gy, gz);
  }

  // ━━━ 상태 스트림 (10Hz) — 라이브 표시용, 포맷 기존과 동일 ━━━
  unsigned long nowMs = millis();
  if (nowMs - lastTx >= TX_INTERVAL_MS) {
    lastTx = nowMs;
    float sAx = peakAx, sAy = peakAy, sAz = peakAz;
    peakMagSq = 0;
    unsigned long hz = sampleCount * 1000UL / TX_INTERVAL_MS;
    sampleCount = 0;

    uint8_t tb[2];
    mpuRead(0x41, tb, 2);                            // TEMP_OUT
    float temp = combine(tb[0], tb[1]) / 340.0 + 36.53;

    int light = analogRead(LIGHT_PIN);
    digitalWrite(RELAY_PIN, light < LIGHT_THRESHOLD ? LOW : HIGH);
    int fsr = analogRead(FSR_PIN);
    int alcohol = analogRead(ALCOHOL_PIN);

    // 형식: ax,ay,az,gx,gy,gz,temp,light,fsr,alcohol\n  (master/TUI 그대로)
    char line[96], tmp[12];
    dtostrf(sAx, 1, 3, line); strcat(line, ",");
    dtostrf(sAy, 1, 3, tmp); strcat(line, tmp); strcat(line, ",");
    dtostrf(sAz, 1, 3, tmp); strcat(line, tmp); strcat(line, ",");
    dtostrf(lastGx, 1, 2, tmp); strcat(line, tmp); strcat(line, ",");
    dtostrf(lastGy, 1, 2, tmp); strcat(line, tmp); strcat(line, ",");
    dtostrf(lastGz, 1, 2, tmp); strcat(line, tmp); strcat(line, ",");
    dtostrf(temp, 1, 1, tmp); strcat(line, tmp); strcat(line, ",");
    itoa(light, tmp, 10); strcat(line, tmp); strcat(line, ",");
    itoa(fsr, tmp, 10); strcat(line, tmp); strcat(line, ",");
    itoa(alcohol, tmp, 10); strcat(line, tmp);

    hc06.println(line);     // 블루투스로 송신
    Serial.print(line);     // 디버깅용
    Serial.print(F("   # sampling "));
    Serial.print(hz);
    Serial.println(F("Hz"));

    // 마스터→슬레이브 명령 수신 (선택)
    if (hc06.available()) Serial.write(hc06.read());
  }
}
