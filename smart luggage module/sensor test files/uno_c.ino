/*
 * ============================================================
 *  우노 C 코드
 *  역할: SHARP 2Y0A21 ×5 거리 측정 + UART 송신
 * ============================================================
 *  핀 연결:
 *    SHARP 1 (전방 중앙)  → A0
 *    SHARP 2 (전방 좌)    → A1
 *    SHARP 3 (전방 우)    → A2
 *    SHARP 4 (측면 좌)    → A3
 *    SHARP 5 (측면 우)    → A4
 *    SHARP VCC            → 5V
 *    SHARP GND            → GND
 *    TX (Pin 1)           → 메가 RX3 (Pin 15)
 *
 *  UART 전송 포맷 (20ms 주기):
 *    S,<D1>,<D2>,<D3>,<D4>,<D5>\n
 *    예) S,45.2,32.1,38.7,55.0,60.3\n
 *
 *    D1~D5: SHARP 1~5 거리값 (cm)
 *           범위 초과(>80cm 또는 <10cm) 시 999.0
 *
 *  [SHARP 2Y0A21 비선형 보정]
 *  ADC 전압 → 거리 변환:
 *    distance(cm) = 27.728 * pow(voltage, -1.2045)
 *    (10~80cm 범위 내 실험적 근사식)
 * ============================================================
 */

#include <math.h>

// ── 핀 설정 ─────────────────────────────────────────────────
const uint8_t SHARP_PINS[5] = {A0, A1, A2, A3, A4};
// ────────────────────────────────────────────────────────────

// ── 설정값 ──────────────────────────────────────────────────
const uint8_t  AVG_SAMPLES  = 8;      // 노이즈 평균화 샘플 수
const float    V_REF        = 5.0;    // 아두이노 기준 전압
const uint16_t ADC_MAX      = 1023;
const float    DIST_MIN     = 10.0;   // SHARP 유효 범위 최소 (cm)
const float    DIST_MAX     = 80.0;   // SHARP 유효 범위 최대 (cm)
const uint16_t SEND_INTERVAL = 20;    // UART 전송 주기 (ms)
// ────────────────────────────────────────────────────────────

// ── ADC → 전압 → 거리 변환 ───────────────────────────────────
float adcToDistance(uint8_t pin) {
  // 노이즈 평균화
  uint32_t sum = 0;
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  float adcVal = (float)sum / AVG_SAMPLES;

  // ADC → 전압
  float voltage = adcVal * V_REF / ADC_MAX;

  if (voltage < 0.1) return 999.0;  // 센서 미연결 또는 범위 초과

  // 전압 → 거리 (SHARP 2Y0A21 근사식)
  float dist = 27.728 * pow(voltage, -1.2045);

  // 유효 범위 벗어나면 999.0 반환
  if (dist < DIST_MIN || dist > DIST_MAX) return 999.0;
  return dist;
}

void setup() {
  Serial.begin(9600);  // 메가로 UART 송신

  for (uint8_t i = 0; i < 5; i++) {
    pinMode(SHARP_PINS[i], INPUT);
  }

  Serial.println("=== Uno C SHARP Ready ===");
}

uint32_t lastSend = 0;

void loop() {
  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();

    float dist[5];
    for (uint8_t i = 0; i < 5; i++) {
      dist[i] = adcToDistance(SHARP_PINS[i]);
    }

    // 전송 포맷: S,<D1>,<D2>,<D3>,<D4>,<D5>\n
    Serial.print('S');
    for (uint8_t i = 0; i < 5; i++) {
      Serial.print(',');
      Serial.print(dist[i], 1);
    }
    Serial.println();
  }
}
