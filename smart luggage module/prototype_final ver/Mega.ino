/*
 * ============================================================
 *  Arduino Mega 2560 — 중앙 통합 제어
 * ============================================================
 *  [역할]
 *  - HX1838 CH3/CH4 (중앙 IR 수신) — 인터럽트 직접 디코딩
 *  - 우노 A/B로부터 UART 수신 (IR채널 + 초음파 + SHARP)
 *  - IR 6채널 무게중심 기반 방향 판별
 *  - SHARP 5채널 사람 인식 (움직임 + 패턴 융합)
 *  - MDD10A 모터 제어 (킥스타트 포함)
 *  - 부저 제어 (홀드 중 짧은 비프 / 소실 시 긴 비프)
 *
 *  [추종 로직]
 *  무게중심 < 10cm → 좌회전
 *  무게중심 = 10cm → 직진 (SHARP 거리 기반 속도)
 *  무게중심 > 10cm → 우회전
 *  IR 소실 → 홀드 500ms (마지막 방향 서행 + 짧은 비프)
 *  홀드 만료 → 정지 + 긴 비프
 *  HC-SR04 장애물 → 회피
 *
 *  [킥스타트]
 *  정지 → 출발 시 150ms 동안 목표 PWM + 40 출력
 *
 *  핀 연결:
 *    HX1838 CH3 OUT → Pin 2  (INT4)
 *    HX1838 CH4 OUT → Pin 3  (INT5)
 *    우노 A TX      → 메가 RX1 (Pin 19)
 *    우노 B TX      → 메가 RX2 (Pin 17)
 *    MDD10A PWM1    → Pin 5
 *    MDD10A DIR1    → Pin 6
 *    MDD10A PWM2    → Pin 7
 *    MDD10A DIR2    → Pin 8
 *    부저           → Pin 11
 *    공통 GND 연결 필수
 * ============================================================
 */

// ── 핀 설정 ─────────────────────────────────────────────────
const uint8_t IR_CH3_PIN = 2;
const uint8_t IR_CH4_PIN = 3;
const uint8_t PWM1_PIN   = 5;
const uint8_t DIR1_PIN   = 6;
const uint8_t PWM2_PIN   = 7;
const uint8_t DIR2_PIN   = 8;
const uint8_t BUZZ_PIN   = 11;
const uint8_t LED_LEFT   = 22;   // 좌회전 LED (녹색)
const uint8_t LED_CENTER = 23;   // 직진 LED (녹색)
const uint8_t LED_RIGHT  = 24;   // 우회전 LED (녹색)
const uint8_t LED_HUMAN  = 25;   // SHARP 사람 감지 LED (빨간색)
const uint8_t LED_OBS_L  = 26;   // 좌측 장애물 감지 LED (황색)
const uint8_t LED_OBS_R  = 27;   // 우측 장애물 감지 LED (황색)

// ── NEC 프로토콜 설정 ────────────────────────────────────────
const uint8_t  VALID_ADDRESS = 0xAA;
const uint8_t  VALID_COMMAND = 0x55;
const uint32_t IR_TIMEOUT_MS = 500;

// ── 패킷 마커 ────────────────────────────────────────────────
const uint8_t PKT_START = 0xAA;
const uint8_t PKT_END   = 0xBB;

// ── 속도 설정 ────────────────────────────────────────────────
const int SPD_FAST   = 90;
const int SPD_NORMAL = 70;
const int SPD_SLOW   = 45;
const int SPD_TURN   = 60;
const int SPD_HOLD   = 40;   // 홀드 중 서행 속도

// ── 거리 임계값 (cm) ─────────────────────────────────────────
const float DIST_FAST   = 50.0;
const float DIST_NORMAL = 20.0;
const float DIST_SLOW   = 10.0;

// ── 무게중심 설정 ────────────────────────────────────────────
const float CH_POS[6]     = {0, 4, 8, 12, 16, 20};  // cm
const float CENTER_TARGET = 10.0;  // 정중앙
const float TURN_DEAD     = 1.5;   // 이 범위 내면 직진 (dead zone)

// ── 홀드 설정 ────────────────────────────────────────────────
const uint32_t HOLD_MS = 500;

// ── 킥스타트 설정 ────────────────────────────────────────────
const int      KICK_BOOST    = 80;
const uint32_t KICK_DURATION = 150;

// ── SHARP 사람 인식 설정 ─────────────────────────────────────
const float    DIST_MIN         = 10.0;
const float    DIST_MAX         = 80.0;
const float    MOTION_THRESHOLD = 2.5;
const float    PATTERN_MARGIN   = 3.0;
const uint8_t  MOTION_MIN_CH    = 2;
const uint8_t  HIST_SIZE        = 16;
const int      HUMAN_THRESHOLD  = 2;
const uint32_t HUMAN_HOLD_MS    = 1000;

// ── NEC 타이밍 허용 범위 ─────────────────────────────────────
#define IN_RANGE(val, target) ((val) > (uint32_t)(target) * 7 / 10 && \
                               (val) < (uint32_t)(target) * 13 / 10)

// ── IR 채널 구조체 ───────────────────────────────────────────
struct IRChannel {
  volatile uint32_t pulses[68];
  volatile uint8_t  idx;
  volatile bool     ready;
  volatile uint32_t lastTime;
  uint32_t          lastValid;
};

IRChannel ch3, ch4;

// ── 전체 IR 채널 상태 (6채널) ────────────────────────────────
bool chActive[6] = {false};  // CH1~CH6

// ── SHARP 거리 데이터 ────────────────────────────────────────
// [0]=S1중앙 [1]=S2좌2nd [2]=S3우2nd [3]=S4좌끝 [4]=S5우끝
float sharpDist[5]                 = {80, 80, 80, 80, 80};
float distHistory[5][HIST_SIZE];
uint8_t histIdx  = 0;
bool    histFull = false;
uint32_t lastHumanTime = 0;

// ── 초음파 거리 (우노 A/B 수신) ─────────────────────────────
uint16_t distA = 999;  // 우노 A 초음파
uint16_t distB = 999;  // 우노 B 초음파

// ── 킥스타트 상태 ────────────────────────────────────────────
bool     wasStop       = true;
uint32_t kickStartTime = 0;

// ── 홀드 상태 ────────────────────────────────────────────────
uint32_t  lastIRTime    = 0;
float     lastCenter    = CENTER_TARGET;

// ── ISR ──────────────────────────────────────────────────────
void handleIR(IRChannel& ch) {
  uint32_t now      = micros();
  uint32_t duration = now - ch.lastTime;
  ch.lastTime = now;
  if (duration > 60000UL) { ch.idx = 0; ch.ready = false; return; }
  if (ch.idx < 68) ch.pulses[ch.idx++] = duration;
  if (ch.idx == 67) ch.ready = true;
}
void ch3ISR() { handleIR(ch3); }
void ch4ISR() { handleIR(ch4); }

// ── NEC 디코딩 ───────────────────────────────────────────────
bool decodeNEC(uint32_t* p, uint8_t& addr, uint8_t& cmd) {
  if (!IN_RANGE(p[0], 9000)) return false;
  if (!IN_RANGE(p[1], 4500)) return false;
  uint32_t raw = 0;
  for (uint8_t i = 0; i < 32; i++) {
    if (!IN_RANGE(p[2 + i*2], 560)) return false;
    if      (IN_RANGE(p[3 + i*2], 1690)) raw |= (1UL << i);
    else if (IN_RANGE(p[3 + i*2],  560)) {}
    else return false;
  }
  uint8_t a = raw & 0xFF, ai = (raw>>8) & 0xFF;
  uint8_t c = (raw>>16) & 0xFF, ci = (raw>>24) & 0xFF;
  if ((a ^ ai) != 0xFF || (c ^ ci) != 0xFF) return false;
  addr = a; cmd = c;
  return true;
}

void processIRChannel(IRChannel& ch, uint8_t chIdx) {
  if (!ch.ready) return;
  noInterrupts();
  uint32_t buf[67];
  for (uint8_t i = 0; i < 67; i++) buf[i] = ch.pulses[i];
  ch.ready = false; ch.idx = 0;
  interrupts();
  uint8_t addr, cmd;
  if (decodeNEC(buf, addr, cmd))
    if (addr == VALID_ADDRESS && cmd == VALID_COMMAND)
      ch.lastValid = millis();
  chActive[chIdx] = (ch.lastValid != 0 &&
                     millis() - ch.lastValid <= IR_TIMEOUT_MS);
}

// ── UART 패킷 수신 (우노 A) ──────────────────────────────────
// 패킷: START | CH1 | CH2 | DIST_H | DIST_L | S4_H | S4_L | S2_H | S2_L | END
void readUnoA() {
  if (Serial1.available() < 10) return;
  if (Serial1.read() != PKT_START) return;

  uint8_t buf[8];
  for (uint8_t i = 0; i < 8; i++) {
    if (!Serial1.available()) return;
    buf[i] = Serial1.read();
  }
  if (!Serial1.available() || Serial1.read() != PKT_END) return;

  chActive[0] = buf[0];  // CH1
  chActive[1] = buf[1];  // CH2
  distA = ((uint16_t)buf[2] << 8) | buf[3];
  sharpDist[3] = ((uint16_t)buf[4] << 8 | buf[5]) / 10.0;  // S4
  sharpDist[1] = ((uint16_t)buf[6] << 8 | buf[7]) / 10.0;  // S2
}

// ── UART 패킷 수신 (우노 B) ──────────────────────────────────
// 패킷: START | CH5 | CH6 | DIST_H | DIST_L | S1_H | S1_L | S3_H | S3_L | S5_H | S5_L | END
void readUnoB() {
  if (Serial2.available() < 12) return;
  if (Serial2.read() != PKT_START) return;

  uint8_t buf[10];
  for (uint8_t i = 0; i < 10; i++) {
    if (!Serial2.available()) return;
    buf[i] = Serial2.read();
  }
  if (!Serial2.available() || Serial2.read() != PKT_END) return;

  chActive[4] = buf[0];  // CH5
  chActive[5] = buf[1];  // CH6
  distB = ((uint16_t)buf[2] << 8) | buf[3];
  sharpDist[0] = ((uint16_t)buf[4] << 8 | buf[5]) / 10.0;  // S1
  sharpDist[2] = ((uint16_t)buf[6] << 8 | buf[7]) / 10.0;  // S3
  sharpDist[4] = ((uint16_t)buf[8] << 8 | buf[9]) / 10.0;  // S5
}

// ── 무게중심 계산 ────────────────────────────────────────────
float calcCenter() {
  float weightedSum = 0;
  int   count       = 0;
  for (uint8_t i = 0; i < 6; i++) {
    if (chActive[i]) {
      weightedSum += CH_POS[i];
      count++;
    }
  }
  if (count == 0) return -1;  // 신호 없음
  return weightedSum / count;
}

// ── SHARP 사람 인식 ──────────────────────────────────────────
int checkMotion() {
  if (!histFull && histIdx < 2) return 0;
  uint8_t prevIdx  = (histIdx == 0) ? HIST_SIZE - 1 : histIdx - 1;
  uint8_t detected = 0;
  for (uint8_t i = 0; i < 5; i++) {
    if (sharpDist[i] < DIST_MIN || sharpDist[i] > DIST_MAX) continue;
    if (distHistory[i][prevIdx] < DIST_MIN ||
        distHistory[i][prevIdx] > DIST_MAX) continue;
    if (abs(sharpDist[i] - distHistory[i][prevIdx]) >= MOTION_THRESHOLD)
      detected++;
  }
  return (detected >= MOTION_MIN_CH) ? 2 : 0;
}

int checkPattern() {
  float s1 = sharpDist[0], s2 = sharpDist[1], s3 = sharpDist[2];
  float s4 = sharpDist[3], s5 = sharpDist[4];
  if (s1 < DIST_MIN || s1 > DIST_MAX) return 0;
  bool centerCloser = (s1 < s2 - PATTERN_MARGIN) && (s1 < s3 - PATTERN_MARGIN);
  bool outerFarther = (s2 < s4 - PATTERN_MARGIN) && (s3 < s5 - PATTERN_MARGIN);
  if (centerCloser && outerFarther) return 1;
  bool onlyCenter = (s1 < DIST_MAX * 0.6f)        &&
                    (s2 > s1 + PATTERN_MARGIN)     &&
                    (s3 > s1 + PATTERN_MARGIN)     &&
                    (s4 > s1 + PATTERN_MARGIN * 2) &&
                    (s5 > s1 + PATTERN_MARGIN * 2);
  return onlyCenter ? 1 : 0;
}

bool isHuman() {
  uint32_t now   = millis();
  int score = checkMotion() + checkPattern();
  if (score >= HUMAN_THRESHOLD) lastHumanTime = now;
  // 히스토리 갱신
  for (uint8_t i = 0; i < 5; i++) distHistory[i][histIdx] = sharpDist[i];
  histIdx = (histIdx + 1) % HIST_SIZE;
  if (histIdx == 0) histFull = true;
  return (now - lastHumanTime <= HUMAN_HOLD_MS);
}

// ── 모터 제어 ────────────────────────────────────────────────
void setMotor(uint8_t pwmPin, uint8_t dirPin, int speed) {
  if (speed > 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, constrain(speed, 0, 255));
  } else if (speed < 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, constrain(-speed, 0, 255));
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopMotors() {
  setMotor(PWM1_PIN, DIR1_PIN, 0);
  setMotor(PWM2_PIN, DIR2_PIN, 0);
  wasStop = true;
}

void forwardWithKick(int speed) {
  uint32_t now = millis();
  if (wasStop) { kickStartTime = now; wasStop = false; }
  int s = (now - kickStartTime < KICK_DURATION) ?
          constrain(speed + KICK_BOOST, 0, 255) : speed;
  setMotor(PWM1_PIN, DIR1_PIN,  s);
  setMotor(PWM2_PIN, DIR2_PIN,  s);
}

void turnLeft(int speed) {
  setMotor(PWM1_PIN, DIR1_PIN, -speed);
  setMotor(PWM2_PIN, DIR2_PIN,  speed);
}

void turnRight(int speed) {
  setMotor(PWM1_PIN, DIR1_PIN,  speed);
  setMotor(PWM2_PIN, DIR2_PIN, -speed);
}

// ── 부저 제어 ────────────────────────────────────────────────
// tone()으로 주파수 발생 → 수동 부저에서 훨씬 큰 소리
// 홀드 중: 2500Hz 짧은 비프 (100ms ON / 100ms OFF)
// 소실 시: 1000Hz 긴 비프  (500ms ON / 500ms OFF)
void updateBuzzer(bool irActive, bool holding) {
  uint32_t now = millis();
  if (irActive) {
    noTone(BUZZ_PIN);
    return;
  }
  if (holding) {
    if ((now / 100) % 2 == 0) tone(BUZZ_PIN, 2500);
    else                       noTone(BUZZ_PIN);
  } else {
    if ((now / 500) % 2 == 0) tone(BUZZ_PIN, 1000);
    else                       noTone(BUZZ_PIN);
  }
}

// ── LED 제어 ─────────────────────────────────────────────────
void setLED(bool left, bool center, bool right) {
  digitalWrite(LED_LEFT,   left   ? HIGH : LOW);
  digitalWrite(LED_CENTER, center ? HIGH : LOW);
  digitalWrite(LED_RIGHT,  right  ? HIGH : LOW);
}

// ── 장애물 감지 ──────────────────────────────────────────────
bool obstacleDetected() {
  return (distA < 20 || distB < 20);
}

void updateObstacleLED() {
  digitalWrite(LED_OBS_L, distA < 20 ? HIGH : LOW);
  digitalWrite(LED_OBS_R, distB < 20 ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);   // 디버그
  Serial1.begin(115200);  // 우노 A
  Serial2.begin(115200);  // 우노 B

  ch3.idx=0; ch3.ready=false; ch3.lastTime=0; ch3.lastValid=0;
  ch4.idx=0; ch4.ready=false; ch4.lastTime=0; ch4.lastValid=0;

  for (uint8_t i = 0; i < 5; i++)
    for (uint8_t j = 0; j < HIST_SIZE; j++)
      distHistory[i][j] = DIST_MAX;

  pinMode(IR_CH3_PIN, INPUT);
  pinMode(IR_CH4_PIN, INPUT);
  pinMode(PWM1_PIN, OUTPUT); pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT); pinMode(DIR2_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(LED_LEFT,   OUTPUT);
  pinMode(LED_CENTER, OUTPUT);
  pinMode(LED_RIGHT,  OUTPUT);
  pinMode(LED_HUMAN,  OUTPUT);
  pinMode(LED_OBS_L,  OUTPUT);
  pinMode(LED_OBS_R,  OUTPUT);

  stopMotors();
  digitalWrite(BUZZ_PIN, LOW);
  setLED(false, false, false);
  digitalWrite(LED_HUMAN, LOW);
  digitalWrite(LED_OBS_L, LOW);
  digitalWrite(LED_OBS_R, LOW);

  attachInterrupt(digitalPinToInterrupt(IR_CH3_PIN), ch3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_CH4_PIN), ch4ISR, CHANGE);

  Serial.println("=== Mega 2560 통합 추종 시스템 Ready ===");
}

void loop() {
  uint32_t now = millis();

  // ── 데이터 수신 ───────────────────────────────────────────
  readUnoA();
  readUnoB();

  // ── 중앙 IR 처리 ──────────────────────────────────────────
  processIRChannel(ch3, 2);  // CH3
  processIRChannel(ch4, 3);  // CH4

  // ── 무게중심 계산 ─────────────────────────────────────────
  float center = calcCenter();
  bool  irActive = (center >= 0);

  if (irActive) lastIRTime = now;

  bool holding = (!irActive && (now - lastIRTime <= HOLD_MS));
  bool lost    = (!irActive && !holding);

  // ── 부저 ──────────────────────────────────────────────────
  updateBuzzer(irActive, holding);

  // ── 사람 인식 ─────────────────────────────────────────────
  bool human = isHuman();
  digitalWrite(LED_HUMAN, human ? HIGH : LOW);

  // ── 장애물 감지 ───────────────────────────────────────────
  updateObstacleLED();
  if (obstacleDetected()) {
    stopMotors();
    Serial.println("[회피] 장애물 감지 → 정지");
    return;
  }

  // ── 추종 로직 ─────────────────────────────────────────────
  if (lost) {
    stopMotors();
    setLED(false, false, false);
    Serial.println("[정지] IR 소실");

  } else if (holding) {
    float c = lastCenter;
    if (c < CENTER_TARGET - TURN_DEAD) {
      turnLeft(SPD_HOLD);
      setLED(true, false, false);
    } else if (c > CENTER_TARGET + TURN_DEAD) {
      turnRight(SPD_HOLD);
      setLED(false, false, true);
    } else {
      forwardWithKick(SPD_HOLD);
      setLED(false, true, false);
    }
    Serial.println("[홀드] 서행 유지");

  } else if (irActive) {
    lastCenter = center;
    float deviation = center - CENTER_TARGET;

    if (deviation < -TURN_DEAD) {
      turnLeft(SPD_TURN);
      setLED(true, false, false);
      Serial.print("[좌회전] center="); Serial.println(center);

    } else if (deviation > TURN_DEAD) {
      turnRight(SPD_TURN);
      setLED(false, false, true);
      Serial.print("[우회전] center="); Serial.println(center);

    } else {
      float dist = sharpDist[0];
      setLED(false, true, false);  // 직진 시 가운데 LED

      if (!human) {
        forwardWithKick(SPD_SLOW);
        Serial.println("[서행] 사람 미인식");
      } else if (dist > DIST_FAST) {
        forwardWithKick(SPD_FAST);
        Serial.print("[빠른 직진] dist="); Serial.println(dist);
      } else if (dist > DIST_NORMAL) {
        forwardWithKick(SPD_NORMAL);
        Serial.print("[보통 직진] dist="); Serial.println(dist);
      } else if (dist > DIST_SLOW) {
        forwardWithKick(SPD_SLOW);
        Serial.print("[느린 직진] dist="); Serial.println(dist);
      } else {
        stopMotors();
        setLED(false, false, false);
        Serial.println("[정지] 근접");
      }
    }
  }

  delay(20);  // 50Hz 루프
}
