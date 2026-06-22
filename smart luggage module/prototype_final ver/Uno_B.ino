/*
 * ============================================================
 *  Arduino Uno B — 우측 IR 수신 + HC-SR04 + SHARP ×3
 * ============================================================
 *  [역할]
 *  - HX1838 CH5/CH6 (우측 IR 수신) — 인터럽트 직접 디코딩
 *  - HC-SR04 (우측 전방 장애물 감지)
 *  - SHARP 2Y0A21 ×3 (S1 중앙, S3 우2번째, S5 우끝) — ADC 읽기
 *  - 20ms 주기로 메가 Serial2에 데이터 송신
 *  - 우노 A와 10ms 오프셋으로 송신 (UART 충돌 방지)
 *
 *  [송신 패킷 구조] 총 12바이트
 *  START(0xAA) | CH5 | CH6 | DIST_H | DIST_L | S1_H | S1_L | S3_H | S3_L | S5_H | S5_L | END(0xBB)
 *
 *  핀 연결:
 *    HX1838 CH5 OUT → Pin 2 (INT0)
 *    HX1838 CH6 OUT → Pin 3 (INT1)
 *    HC-SR04 TRIG   → Pin 7
 *    HC-SR04 ECHO   → Pin 8
 *    SHARP S1 (중앙)  → A0
 *    SHARP S3 (우2nd) → A1
 *    SHARP S5 (우끝)  → A2
 *    메가 RX2 (Pin 17) → 우노 B TX (Pin 1)
 *    공통 GND 연결 필수
 * ============================================================
 */

// ── 핀 설정 ─────────────────────────────────────────────────
const uint8_t IR_CH5_PIN  = 2;
const uint8_t IR_CH6_PIN  = 3;
const uint8_t TRIG_PIN    = 7;
const uint8_t ECHO_PIN    = 8;
const uint8_t SHARP_S1    = A0;
const uint8_t SHARP_S3    = A1;
const uint8_t SHARP_S5    = A2;

// ── NEC 프로토콜 설정 ────────────────────────────────────────
const uint8_t  VALID_ADDRESS = 0xAA;
const uint8_t  VALID_COMMAND = 0x55;
const uint32_t IR_TIMEOUT_MS = 500;

// ── 송신 설정 ────────────────────────────────────────────────
const uint32_t SEND_INTERVAL  = 20;   // ms
const uint32_t SEND_OFFSET    = 10;   // 우노 A와 10ms 엇갈림

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

IRChannel ch5, ch6;

// ── ISR ──────────────────────────────────────────────────────
void handleIR(IRChannel& ch) {
  uint32_t now      = micros();
  uint32_t duration = now - ch.lastTime;
  ch.lastTime = now;
  if (duration > 60000UL) { ch.idx = 0; ch.ready = false; return; }
  if (ch.idx < 68) ch.pulses[ch.idx++] = duration;
  if (ch.idx == 67) ch.ready = true;
}
void ch5ISR() { handleIR(ch5); }
void ch6ISR() { handleIR(ch6); }

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
void sendPacket(bool ch5Active, bool ch6Active,
                uint16_t dist, float s1, float s3, float s5) {
  uint16_t s1i = (uint16_t)(s1 * 10);
  uint16_t s3i = (uint16_t)(s3 * 10);
  uint16_t s5i = (uint16_t)(s5 * 10);

  Serial.write(PKT_START);
  Serial.write((uint8_t)ch5Active);
  Serial.write((uint8_t)ch6Active);
  Serial.write((uint8_t)(dist >> 8));
  Serial.write((uint8_t)(dist & 0xFF));
  Serial.write((uint8_t)(s1i >> 8));
  Serial.write((uint8_t)(s1i & 0xFF));
  Serial.write((uint8_t)(s3i >> 8));
  Serial.write((uint8_t)(s3i & 0xFF));
  Serial.write((uint8_t)(s5i >> 8));
  Serial.write((uint8_t)(s5i & 0xFF));
  Serial.write(PKT_END);
}

void setup() {
  Serial.begin(115200);

  ch5.idx=0; ch5.ready=false; ch5.lastTime=0; ch5.lastValid=0;
  ch6.idx=0; ch6.ready=false; ch6.lastTime=0; ch6.lastValid=0;

  pinMode(IR_CH5_PIN, INPUT);
  pinMode(IR_CH6_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(IR_CH5_PIN), ch5ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_CH6_PIN), ch6ISR, CHANGE);

  // 10ms 오프셋 — 우노 A(0ms 기준)와 엇갈려서 UART 충돌 방지
  delay(SEND_OFFSET);
}

void loop() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();

  processChannel(ch5);
  processChannel(ch6);

  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;

    bool ch5Active = (ch5.lastValid != 0 && now - ch5.lastValid <= IR_TIMEOUT_MS);
    bool ch6Active = (ch6.lastValid != 0 && now - ch6.lastValid <= IR_TIMEOUT_MS);

    uint16_t dist = measureUltrasonic();
    float s1 = readSharp(SHARP_S1);
    float s3 = readSharp(SHARP_S3);
    float s5 = readSharp(SHARP_S5);

    sendPacket(ch5Active, ch6Active, dist, s1, s3, s5);
  }
}
