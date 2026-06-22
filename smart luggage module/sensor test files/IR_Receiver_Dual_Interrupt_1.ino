/*
 * ============================================================
 *  IR 수신기 코드 — Arduino Mega 2560 (인터럽트 직접 디코딩)
 *  수신 센서 : CN6 1838 × 2 (왼쪽 / 오른쪽) 완전 독립 동시 수신
 * ============================================================
 *  왼쪽  센서 수신 성공 → 왼쪽  녹색 LED ON
 *  오른쪽 센서 수신 성공 → 오른쪽 녹색 LED ON
 *  타임아웃 시 해당 LED OFF
 *
 *  [NEC 프로토콜 타이밍 기준]
 *  - 리더 마크   : 9000µs
 *  - 리더 스페이스: 4500µs
 *  - 비트 마크   :  560µs
 *  - '0' 스페이스:  560µs
 *  - '1' 스페이스: 1690µs
 *  - 허용 오차   : ±30%
 *
 *  핀 연결:
 *    왼쪽  CN6 1838 OUT → Pin 2
 *    오른쪽 CN6 1838 OUT → Pin 3
 *    CN6 1838 VCC        → 5V
 *    CN6 1838 GND        → GND
 *    왼쪽  녹색 LED (+)  → Pin 6  (220Ω 직렬)
 *    오른쪽 녹색 LED (+) → Pin 7  (220Ω 직렬)
 *    LED (-)             → GND
 * ============================================================
 */

// ── 핀 설정 ─────────────────────────────────────────────────
const uint8_t IR_LEFT_PIN  = 2;
const uint8_t IR_RIGHT_PIN = 3;
const uint8_t LED_LEFT     = 6;
const uint8_t LED_RIGHT    = 7;

// ── 유효 신호 설정 ───────────────────────────────────────────
const uint8_t  VALID_ADDRESS = 0xAA;
const uint8_t  VALID_COMMAND = 0x55;
const uint32_t TIMEOUT_MS    = 500;

// ── NEC 타이밍 허용 범위 (±30%) ─────────────────────────────
#define IN_RANGE(val, target) ((val) > (uint32_t)(target) * 7 / 10 && \
                               (val) < (uint32_t)(target) * 13 / 10)

// ── 채널 구조체 ──────────────────────────────────────────────
struct IRChannel {
  volatile uint32_t pulses[68];
  volatile uint8_t  idx;
  volatile bool     ready;
  volatile uint32_t lastTime;
  uint32_t          lastValid;
};

// 지정 초기화자 대신 일반 초기화로 변경 (AVR 호환)
IRChannel chLeft;
IRChannel chRight;

// ── ISR 전방 선언 ────────────────────────────────────────────
void handleIR(IRChannel& ch);
void leftISR();
void rightISR();

// ── 공용 ISR 처리 함수 ───────────────────────────────────────
void handleIR(IRChannel& ch) {
  uint32_t now      = micros();
  uint32_t duration = now - ch.lastTime;
  ch.lastTime = now;

  // 60ms 이상 = 새 프레임 시작
  if (duration > 60000UL) {
    ch.idx   = 0;
    ch.ready = false;
    return;
  }

  if (ch.idx < 68) {
    ch.pulses[ch.idx++] = duration;
  }

  // NEC: 리더 2펄스 + 32비트×2펄스 + 스톱 1펄스 = 67펄스
  if (ch.idx == 67) {
    ch.ready = true;
  }
}

void leftISR()  { handleIR(chLeft);  }
void rightISR() { handleIR(chRight); }

// ── NEC 디코딩 함수 ──────────────────────────────────────────
bool decodeNEC(uint32_t* pulses, uint8_t& address, uint8_t& command) {
  if (!IN_RANGE(pulses[0], 9000)) return false;
  if (!IN_RANGE(pulses[1], 4500)) return false;

  uint32_t raw = 0;

  for (uint8_t i = 0; i < 32; i++) {
    uint8_t markIdx  = 2 + i * 2;
    uint8_t spaceIdx = 3 + i * 2;

    if (!IN_RANGE(pulses[markIdx], 560)) return false;

    if (IN_RANGE(pulses[spaceIdx], 1690)) {
      raw |= (1UL << i);  // '1' 비트 (LSB first)
    } else if (IN_RANGE(pulses[spaceIdx], 560)) {
      // '0' 비트
    } else {
      return false;
    }
  }

  uint8_t addr    = (raw >>  0) & 0xFF;
  uint8_t addrInv = (raw >>  8) & 0xFF;
  uint8_t cmd     = (raw >> 16) & 0xFF;
  uint8_t cmdInv  = (raw >> 24) & 0xFF;

  // 역코드 검증
  if ((addr ^ addrInv) != 0xFF) return false;
  if ((cmd  ^ cmdInv)  != 0xFF) return false;

  address = addr;
  command = cmd;
  return true;
}

// ── 채널 처리 함수 ───────────────────────────────────────────
void processChannel(IRChannel& ch, uint8_t ledPin, const char* label) {
  if (!ch.ready) return;

  // ISR 공유 데이터 복사 (인터럽트 잠시 중단)
  noInterrupts();
  uint32_t pulsesCopy[67];
  for (uint8_t i = 0; i < 67; i++) pulsesCopy[i] = ch.pulses[i];
  ch.ready = false;
  ch.idx   = 0;
  interrupts();

  uint8_t addr, cmd;
  if (decodeNEC(pulsesCopy, addr, cmd)) {
    Serial.print("["); Serial.print(label); Serial.print("]");
    Serial.print(" Addr=0x"); Serial.print(addr, HEX);
    Serial.print(" Cmd=0x");  Serial.print(cmd,  HEX);

    if (addr == VALID_ADDRESS && cmd == VALID_COMMAND) {
      ch.lastValid = millis();
      Serial.println(" -> VALID");
    } else {
      Serial.println(" -> mismatch");
    }
  }
}

void setup() {
  Serial.begin(9600);

  // 채널 초기화 (AVR은 전역 구조체가 자동으로 0으로 초기화되지만 명시적으로 설정)
  chLeft.idx       = 0;  chLeft.ready  = false;
  chLeft.lastTime  = 0;  chLeft.lastValid  = 0;
  chRight.idx      = 0;  chRight.ready = false;
  chRight.lastTime = 0;  chRight.lastValid = 0;

  pinMode(IR_LEFT_PIN,  INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(LED_LEFT,  OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  digitalWrite(LED_LEFT,  LOW);
  digitalWrite(LED_RIGHT, LOW);

  attachInterrupt(digitalPinToInterrupt(IR_LEFT_PIN),  leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_RIGHT_PIN), rightISR, CHANGE);

  Serial.println("=== Dual IR Receiver (Interrupt) Ready ===");
  Serial.print("Valid Address: 0x"); Serial.println(VALID_ADDRESS, HEX);
  Serial.print("Valid Command: 0x"); Serial.println(VALID_COMMAND, HEX);
}

void loop() {
  processChannel(chLeft,  LED_LEFT,  "LEFT ");
  processChannel(chRight, LED_RIGHT, "RIGHT");

  uint32_t now = millis();
  digitalWrite(LED_LEFT,  (now - chLeft.lastValid  <= TIMEOUT_MS) ? HIGH : LOW);
  digitalWrite(LED_RIGHT, (now - chRight.lastValid <= TIMEOUT_MS) ? HIGH : LOW);
}
