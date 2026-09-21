/*
 * ============================================================
 *  Arduino Uno A — 좌측 IR 수신 + HC-SR04 + SHARP ×2
 * ============================================================
 *  [역할]
 *  - HX1838 CH1/CH2 (좌측 IR 수신) — 인터럽트 직접 디코딩
 *  - HC-SR04 (좌측 전방 장애물 감지)
 *  - SHARP 2Y0A21 ×2 (S4 좌끝, S2 좌2번째) — ADC 읽기
 *  - 20ms 주기로 메가 Serial1에 데이터 송신
 *
 *  [송신 패킷 구조] 총 10바이트
 *  START(0xAA) | CH1 | CH2 | DIST_H | DIST_L | S4_H | S4_L | S2_H | S2_L | END(0xBB)
 *  CH1/CH2: 1=수신중, 0=소실
 *  DIST: 초음파 거리 cm (uint16)
 *  S4/S2: SHARP 거리 cm*10 (uint16, 소수점 1자리)
 *
 *  핀 연결:
 *    HX1838 CH1 OUT → Pin 2 (INT0)
 *    HX1838 CH2 OUT → Pin 3 (INT1)
 *    HC-SR04 TRIG   → Pin 7
 *    HC-SR04 ECHO   → Pin 8
 *    SHARP S4 (좌끝)  → A0
 *    SHARP S2 (좌2nd) → A1
 *    메가 RX1 (Pin 19) → 우노 A TX (Pin 1)
 *    공통 GND 연결 필수
 * ============================================================
 */

// ── 핀 설정 ─────────────────────────────────────────────────
const uint8_t IR_CH1_PIN  = 2;
const uint8_t IR_CH2_PIN  = 3;
const uint8_t TRIG_PIN    = 7;
const uint8_t ECHO_PIN    = 8;
const uint8_t SHARP_S4    = A0;
const uint8_t SHARP_S2    = A1;

// ── NEC 프로토콜 설정 ────────────────────────────────────────
const uint8_t  VALID_ADDRESS = 0xAA;
const uint8_t  VALID_COMMAND = 0x55;
const uint32_t IR_TIMEOUT_MS = 500;

// ── 송신 주기 ────────────────────────────────────────────────
const uint32_t SEND_INTERVAL = 20;  // ms (50Hz)

// ── 패킷 마커 ────────────────────────────────────────────────
const uint8_t PKT_START = 0xAA;
const uint8_t PKT_END   = 0xBB;

// ── NEC 타이밍 허용 범위 (±30%) ─────────────────────────────
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

IRChannel ch1, ch2;

// ── ISR ──────────────────────────────────────────────────────
void handleIR(IRChannel& ch) {
  uint32_t now      = micros();
  uint32_t duration = now - ch.lastTime;
  ch.lastTime = now;
  if (duration > 60000UL) { ch.idx = 0; ch.ready = false; return; }
  if (ch.idx < 68) ch.pulses[ch.idx++] = duration;
  if (ch.idx == 67) ch.ready = true;
}
void ch1ISR() { handleIR(ch1); }
void ch2ISR() { handleIR(ch2); }

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

// ── 채널 처리 ────────────────────────────────────────────────
void processChannel(IRChannel& ch) {
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
}

// ── 초음파 거리 측정 ─────────────────────────────────────────
uint16_t measureUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 999;
  return (uint16_t)(dur * 0.01715);
}

// ── SHARP ADC → 거리 변환 (룩업테이블) ──────────────────────
const uint8_t LUT_SIZE = 10;
const float LUT_ADC[LUT_SIZE]  = {600,530,450,390,330,280,240,210,185,165};
const float LUT_DIST[LUT_SIZE] = { 10, 12, 15, 20, 25, 30, 40, 50, 60, 80};

float adcToDist(int adc) {
  if (adc >= LUT_ADC[0])            return 10.0;
  if (adc <= LUT_ADC[LUT_SIZE - 1]) return 80.0;
  for (uint8_t i = 0; i < LUT_SIZE - 1; i++) {
    if (adc <= LUT_ADC[i] && adc >= LUT_ADC[i+1]) {
      float r = (float)(adc - LUT_ADC[i+1]) / (LUT_ADC[i] - LUT_ADC[i+1]);
      return LUT_DIST[i+1] + r * (LUT_DIST[i] - LUT_DIST[i+1]);
    }
  }
  return 80.0;
}

float readSharp(uint8_t pin) {
  long sum = 0;
  for (uint8_t i = 0; i < 10; i++) { sum += analogRead(pin); delayMicroseconds(200); }
  return adcToDist(sum / 10);
}

// ── 패킷 송신 ────────────────────────────────────────────────
void sendPacket(bool ch1Active, bool ch2Active,
                uint16_t dist, float s4, float s2) {
  uint16_t s4i = (uint16_t)(s4 * 10);
  uint16_t s2i = (uint16_t)(s2 * 10);

  Serial.write(PKT_START);
  Serial.write((uint8_t)ch1Active);
  Serial.write((uint8_t)ch2Active);
  Serial.write((uint8_t)(dist >> 8));
  Serial.write((uint8_t)(dist & 0xFF));
  Serial.write((uint8_t)(s4i >> 8));
  Serial.write((uint8_t)(s4i & 0xFF));
  Serial.write((uint8_t)(s2i >> 8));
  Serial.write((uint8_t)(s2i & 0xFF));
  Serial.write(PKT_END);
}

void setup() {
  Serial.begin(115200);  // 메가 Serial1과 통신

  ch1.idx=0; ch1.ready=false; ch1.lastTime=0; ch1.lastValid=0;
  ch2.idx=0; ch2.ready=false; ch2.lastTime=0; ch2.lastValid=0;

  pinMode(IR_CH1_PIN, INPUT);
  pinMode(IR_CH2_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(IR_CH1_PIN), ch1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_CH2_PIN), ch2ISR, CHANGE);
}

void loop() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();

  // IR 채널 처리
  processChannel(ch1);
  processChannel(ch2);

  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;

    bool ch1Active = (ch1.lastValid != 0 && now - ch1.lastValid <= IR_TIMEOUT_MS);
    bool ch2Active = (ch2.lastValid != 0 && now - ch2.lastValid <= IR_TIMEOUT_MS);

    uint16_t dist = measureUltrasonic();
    float s4 = readSharp(SHARP_S4);
    float s2 = readSharp(SHARP_S2);

    sendPacket(ch1Active, ch2Active, dist, s4, s2);
  }
}
