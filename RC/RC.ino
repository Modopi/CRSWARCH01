/*
 * RC - 적외선 리모컨으로 움직이는 스텝모터 RC카
 * ----------------------------------------------------------
 * 하드웨어
 *   - Arduino UNO/Nano
 *   - 28BYJ-48 스텝모터 2개 (좌/우 바퀴)
 *   - ULN2003 드라이버 모듈 2개 (IN1~IN4)
 *   - IR 수신부 (VS1838B 등) + "Car MP3" 17버튼 리모컨
 *
 * 필요한 라이브러리 (Arduino IDE > 라이브러리 매니저에서 설치)
 *   - AccelStepper  (by Mike McCauley)
 *   - IRremote      (by shirriff / Armin Joachimsmeyer, v4.x)
 *
 * 조작 (Car MP3 리모컨)
 *   ▲ (위)      : 전진
 *   ▼ (아래)    : 후진
 *   ◀ (왼쪽)    : 좌회전 (제자리)
 *   ▶ (오른쪽)  : 우회전 (제자리)
 *   OK / 정지   : 정지
 *   숫자 1 / 2  : 속도 -, +
 * ----------------------------------------------------------
 */

#include <AccelStepper.h>
#include <IRremote.hpp>

// ---------- 핀 설정 ----------
// ULN2003 의 핀 순서는 IN1, IN3, IN2, IN4 로 넣어야 회전 방향이 맞습니다.
// 왼쪽 모터 (ULN2003 #1)
#define L_IN1 2
#define L_IN2 3
#define L_IN3 4
#define L_IN4 5
// 오른쪽 모터 (ULN2003 #2)
#define R_IN1 8
#define R_IN2 9
#define R_IN3 10
#define R_IN4 11
// IR 수신부 신호핀
#define IR_PIN 6

AccelStepper motorL(AccelStepper::HALF4WIRE, L_IN1, L_IN3, L_IN2, L_IN4);
AccelStepper motorR(AccelStepper::HALF4WIRE, R_IN1, R_IN3, R_IN2, R_IN4);

// ---------- 속도 ----------
float speed = 700.0;            // 현재 속도 (steps/sec)
const float SPEED_MIN = 200.0;
const float SPEED_MAX = 1000.0; // 28BYJ-48 은 보통 1000 근처가 한계
const float SPEED_STEP = 100.0;

// ---------- Car MP3 리모컨 코드 (NEC) ----------
// 리모컨/IRremote 버전에 따라 값이 다를 수 있으니, 아래 DEBUG 로 확인 후 수정하세요.
#define CMD_UP     0x18   // ▲ 전진
#define CMD_DOWN   0x52   // ▼ 후진
#define CMD_LEFT   0x08   // ◀ 좌회전
#define CMD_RIGHT  0x5A   // ▶ 우회전
#define CMD_OK     0x1C   // OK 정지
#define CMD_1      0x45   // 속도 -
#define CMD_2      0x46   // 속도 +

// ---------- 주행 상태 ----------
enum Drive { STOP, FWD, BACK, TURN_L, TURN_R };
Drive drive = STOP;

unsigned long lastIR = 0;       // 마지막 신호 시각
const unsigned long IR_TIMEOUT = 250;  // 이 시간 동안 신호 없으면 정지(누름 유지 감지)

void applyDrive() {
  // 모터가 마주보게 장착되므로 한쪽은 부호를 반대로 줍니다.
  switch (drive) {
    case FWD:    motorL.setSpeed( speed); motorR.setSpeed(-speed); break;
    case BACK:   motorL.setSpeed(-speed); motorR.setSpeed( speed); break;
    case TURN_L: motorL.setSpeed(-speed); motorR.setSpeed(-speed); break;
    case TURN_R: motorL.setSpeed( speed); motorR.setSpeed( speed); break;
    case STOP:   motorL.setSpeed(0);      motorR.setSpeed(0);      break;
  }
}

void setup() {
  Serial.begin(115200);
  motorL.setMaxSpeed(SPEED_MAX);
  motorR.setMaxSpeed(SPEED_MAX);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  Serial.println(F("RC ready. Car MP3 remote."));
}

void loop() {
  // ----- IR 수신 -----
  if (IrReceiver.decode()) {
    uint16_t cmd = IrReceiver.decodedIRData.command;
    bool repeat = IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT;

    // 코드 확인용 디버그 (시리얼 모니터 115200)
    Serial.print(F("cmd=0x")); Serial.println(cmd, HEX);

    if (!repeat) {   // 새 버튼
      switch (cmd) {
        case CMD_UP:    drive = FWD;    break;
        case CMD_DOWN:  drive = BACK;   break;
        case CMD_LEFT:  drive = TURN_L; break;
        case CMD_RIGHT: drive = TURN_R; break;
        case CMD_OK:    drive = STOP;   break;
        case CMD_2:     speed = min(speed + SPEED_STEP, SPEED_MAX); break;
        case CMD_1:     speed = max(speed - SPEED_STEP, SPEED_MIN); break;
      }
      applyDrive();
    }
    lastIR = millis();
    IrReceiver.resume();   // 다음 신호 받기
  }

  // ----- 버튼에서 손을 떼면(반복신호 끊김) 정지 -----
  if (drive != STOP && millis() - lastIR > IR_TIMEOUT) {
    drive = STOP;
    applyDrive();
  }

  // ----- 모터 구동 (논블로킹, 매 루프 호출 필수) -----
  motorL.runSpeed();
  motorR.runSpeed();
}
