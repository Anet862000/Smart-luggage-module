/*
 * ============================================================
 *  IR 송신기 코드 — Arduino Uno + KY-005
 * ============================================================
 *  프로토콜 : NEC (38kHz 캐리어)
 *  Address  : 0xAA (고정)
 *  Command  : 0x55 (유효 신호)
 *
 *  [코드 선택 이유]
 *  - NEC 프로토콜은 38kHz 캐리어로 형광등 간섭을 하드웨어 레벨에서 차단
 *  - 0xAA(10101010), 0x55(01010101) : 1과 0이 교번 → 피크 전류 분산,
 *    전파 균일, 형광등 노이즈(저주파 DC 성분)와 패턴이 달라 오수신 최소화
 *  - 0x00, 0xFF처럼 비트가 편중된 코드는 피함 (NEC inverse 검증에 불리)
 *
 *  라이브러리 : IRremote (v4.x)
 *    설치: Arduino IDE > 라이브러리 관리 > "IRremote" 검색 > 설치
 *
 *  핀 연결:
 *    KY-005 Signal → Pin 3 (IRremote 기본 송신 핀)
 *    KY-005 VCC    → 5V
 *    KY-005 GND    → GND
 * ============================================================
 */

#include <IRremote.hpp>

// ── 설정 ────────────────────────────────────────────────────
const uint8_t  IR_TX_PIN   = 3;       // KY-005 신호 핀
const uint16_t IR_ADDRESS  = 0x00AA;  // NEC Address (상위 바이트 0x00 고정)
const uint8_t  IR_COMMAND  = 0x55;    // 유효 신호 코드 (01010101)
const uint16_t SEND_INTERVAL_MS = 150; // 전송 주기 (ms) — 너무 빠르면 수신 버퍼 포화
// ────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);
  IrSender.begin(IR_TX_PIN);

  Serial.println("=== IR Transmitter Ready ===");
  Serial.print("Address : 0x");
  Serial.println(IR_ADDRESS, HEX);
  Serial.print("Command : 0x");
  Serial.println(IR_COMMAND, HEX);
  Serial.print("Interval: ");
  Serial.print(SEND_INTERVAL_MS);
  Serial.println(" ms");
}

void loop() {
  // NEC 프로토콜로 전송 (마지막 인자 0 = repeat 없음)
  IrSender.sendNEC(IR_ADDRESS, IR_COMMAND, 0);

  Serial.print("[TX] NEC 0x");
  Serial.print(IR_ADDRESS, HEX);
  Serial.print(" / 0x");
  Serial.println(IR_COMMAND, HEX);

  delay(SEND_INTERVAL_MS);
}
