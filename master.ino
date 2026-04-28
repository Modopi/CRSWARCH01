#include <SoftwareSerial.h>
#include <math.h>

//zs 040
SoftwareSerial hc06(2, 3);

// ── 파라미터 ──
const float HEAD_MASS    = 5.0;    // 머리+헬멧 추정 질량 (kg)
const float G            = 9.807;  // m/s²
const float IMPACT_THRESH = 2.5;   // 충격 판정 임계 (g)

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

  Serial.println(F("╔══════════════════════════════════════╗"));
  Serial.println(F("║   Smart Helmet  —  Impact Monitor    ║"));
  Serial.println(F("╚══════════════════════════════════════╝"));
  Serial.println();
}

void loop() {
  if (!hc06.available()) return;

  String line = hc06.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // ax, ay, az, gx, gy, gz, temp
  float v[7];
  if (!parseCSV(line, v, 7)) return;

  float ax = v[0], ay = v[1], az = v[2];   // g
  float gx = v[3], gy = v[4], gz = v[5];   // °/s
  float temp = v[6];                         // °C

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

  // ━━━━━━ 출력 ━━━━━━
  Serial.println(F("┌──────────────────────────────────────┐"));

  // 충격 경고
  if (impact) {
    Serial.println(F("│  ⚠⚠⚠   IMPACT DETECTED   ⚠⚠⚠       │"));
    Serial.println(F("├──────────────────────────────────────┤"));
  }

  // 가속도
  Serial.print(F("│ Accel   "));
  Serial.print(ax, 3); Serial.print(F("  "));
  Serial.print(ay, 3); Serial.print(F("  "));
  Serial.print(az, 3); Serial.println(F(" g"));

  Serial.print(F("│ |a|    "));
  Serial.print(aMag, 3); Serial.print(F(" g    net "));
  Serial.print(aNet, 3); Serial.println(F(" g"));

  // 힘 · 충격량
  Serial.print(F("│ Force   "));
  Serial.print(force, 2); Serial.println(F(" N"));

  Serial.print(F("│ Impulse "));
  Serial.print(impulse, 4); Serial.println(F(" N·s"));

  Serial.print(F("│ Jerk    "));
  Serial.print(jerk, 2); Serial.println(F(" g/s"));

  // 자세
  Serial.println(F("├──────────────────────────────────────┤"));

  Serial.print(F("│ Roll    "));
  Serial.print(roll, 1); Serial.print(F("°   Pitch "));
  Serial.print(pitch, 1); Serial.println(F("°"));

  // 자이로
  Serial.print(F("│ Gyro    "));
  Serial.print(gx, 1); Serial.print(F("  "));
  Serial.print(gy, 1); Serial.print(F("  "));
  Serial.print(gz, 1); Serial.println(F(" °/s"));

  Serial.print(F("│ |ω|    "));
  Serial.print(gyroMag, 1); Serial.println(F(" °/s"));

  // 온도 · 피크
  Serial.println(F("├──────────────────────────────────────┤"));

  Serial.print(F("│ Temp    "));
  Serial.print(temp, 1); Serial.println(F(" °C"));

  Serial.print(F("│ Peak    "));
  Serial.print(peakG, 3); Serial.print(F(" g  /  "));
  Serial.print(peakForce, 1); Serial.println(F(" N"));

  Serial.println(F("└──────────────────────────────────────┘"));
  Serial.println();

  // ── 다음 프레임 준비 ──
  prevAx = ax; prevAy = ay; prevAz = az;
  prevMag = aMag;
  firstFrame = false;
}
