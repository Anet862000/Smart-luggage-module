/*
 * ============================================================
 *  우노 A / 우노 B 공용 코드
 *  역할: IR 수신 (HX1838 ×2) + HC-SR04 장애물 감지 + UART 송신
 * ============================================================
 *  [우노 A 설정]
 *    #define BOARD_SIDE LEFT
 *
 *  [우노 B 설정]
 *    #define BOARD_SIDE RIGHT
 *
 *  핀 연결:
 *    HX1838 왼쪽  OUT → Pin 2 (INT0)
 *    HX1838 오른쪽 OUT → Pin 3 (INT1)
 *    HX1838 VCC        → 5V
 *    HX1838 GND        → GND
 *    HC-SR04 TRIG      → Pin 9
 *    HC-SR04 ECHO      → Pin 10
 *    HC-SR04 VCC       → 5V
 *    HC-SR04 GND       → GND
 *    TX (Pin 1)        → 메가 RX1(우노A) 또는 RX2(우노B)
 *
 *  UART 전송 포맷 (20ms 주기):
 *    <SIDE>,<IR_LEFT>,<IR_RIGHT>,<DIST>\n
 *    예) L,1,0,35.2\n
 *        R,0,1,120.0\n
 *
 *    SIDE     : L(우노A) / R(우노B)
 *    IR_LEFT  : 왼쪽  HX1838 수신 여부 (1=유효, 0=없음)
 *    IR_RIGHT : 오른쪽 HX1838 수신 여부 (1=유효, 0=없음)
 *    DIST     : HC-SR04 거리 (cm), 범위 초과 시 999.0
 * ============================================================
 */

// ── 보드 설정 ─────────────────────────────────────────────
#define BOARD_SIDE LEFT   // 우노 A: LEFT / 우노 B: RIGHT
// ────────────────────────────────────────────────────────────

// ── 핀 설정 ─────────────────────────────────────────────────
const uint8_t IR_PIN_L  = 2;    // HX1838 왼쪽  (INT0)
const uint8_t IR_PIN_R  = 3;    // HX1838 오른쪽 (INT1)
const uint8_t TRIG_PIN  = 9;
const uint8_t ECHO_PIN  = 10;
// ────────────────────────────────────────────────────────────

// ── 유효 신호 설정 ───────────────────────────────────────────
const uint8_t  VALID_ADDRESS = 0xAA;
const uint8_t  VALID_COMMAND = 0x55;
const uint32_t IR_TIMEOUT_MS = 500;   // 신호 소실 판정 (ms)
const uint16_t SEND_INTERVAL = 20;    // UART 전송 주기 (ms)
// ────────────────────────────────────────────────────────────

// ── NEC 타이밍 허용 범위 (±30%) ─────────────────────────────
#define IN_RANGE(val, target) \
  ((val) > (uint32_t)(target) * 7 / 10 && \
   (val) < (uint32_t)(target) * 13 / 10)

// ── IR 채널 구조체 ───────────────────────────────────────────
struct IRChannel {
  volatile uint32_t pulses[68];
  volatile uint8_t  idx;
  volatile bool     ready;
  volatile uint32_t lastEdge;
  uint32_t          lastValid;
  bool              state;        // 현재 수신 상태 (true=유효)
};

IRChannel irL, irR;

// ── ISR 처리 ────────────────────────────────────────────────
void handleIR(IRChannel& ch) {
  uint32_t now      = micros();
  uint32_t duration = now - ch.lastEdge;
  ch.lastEdge = now;

  if (duration > 60000UL) { ch.idx = 0; ch.ready = false; return; }
  if (ch.idx < 68) ch.pulses[ch.idx++] = duration;
  if (ch.idx == 67) ch.ready = true;
}

void isrL() { handleIR(irL); }
void isrR() { handleIR(irR); }

// ── NEC 디코딩 ───────────────────────────────────────────────
bool decodeNEC(volatile uint32_t* p, uint8_t& addr, uint8_t& cmd) {
  if (!IN_RANGE(p[0], 9000)) return false;
  if (!IN_RANGE(p[1], 4500)) return false;

  uint32_t raw = 0;
  for (uint8_t i = 0; i < 32; i++) {
    if (!IN_RANGE(p[2 + i*2], 560)) return false;
    if      (IN_RANGE(p[3 + i*2], 1690)) raw |= (1UL << i);
    else if (!IN_RANGE(p[3 + i*2], 560)) return false;
  }

  uint8_t a = raw & 0xFF, ai = (raw >> 8) & 0xFF;
  uint8_t c = (raw >> 16) & 0xFF, ci = (raw >> 24) & 0xFF;
  if ((a ^ ai) != 0xFF || (c ^ ci) != 0xFF) return false;

  addr = a; cmd = c;
  return true;
}

// ── IR 채널 처리 ─────────────────────────────────────────────
void processIR(IRChannel& ch) {
  if (!ch.ready) return;

  noInterrupts();
  uint32_t buf[67];
  for (uint8_t i = 0; i < 67; i++) buf[i] = ch.pulses[i];
  ch.ready = false;
  ch.idx   = 0;
  interrupts();

  uint8_t addr, cmd;
  if (decodeNEC(buf, addr, cmd)) {
    if (addr == VALID_ADDRESS && cmd == VALID_COMMAND) {
      ch.lastValid = millis();
    }
  }
  ch.state = (millis() - ch.lastValid <= IR_TIMEOUT_MS);
}

// ── 초음파 거리 측정 ─────────────────────────────────────────
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 999.0;
  return dur * 0.01715;
}

// ── setup / loop ─────────────────────────────────────────────
void setup() {
  Serial.begin(9600);  // 메가로 UART 송신

  irL = {.idx=0, .ready=false, .lastEdge=0, .lastValid=0, .state=false};
  irR = {.idx=0, .ready=false, .lastEdge=0, .lastValid=0, .state=false};

  pinMode(IR_PIN_L, INPUT);
  pinMode(IR_PIN_R, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(IR_PIN_L), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_PIN_R), isrR, CHANGE);
}

uint32_t lastSend = 0;

void loop() {
  processIR(irL);
  processIR(irR);

  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();

    float dist = measureDistance();

#ifdef BOARD_SIDE
  #if BOARD_SIDE == LEFT
    char side = 'L';
  #else
    char side = 'R';
  #endif
#endif

    // 전송 포맷: <SIDE>,<IR_L>,<IR_R>,<DIST>\n
    Serial.print(side);
    Serial.print(',');
    Serial.print(irL.state ? 1 : 0);
    Serial.print(',');
    Serial.print(irR.state ? 1 : 0);
    Serial.print(',');
    Serial.println(dist, 1);
  }
}
