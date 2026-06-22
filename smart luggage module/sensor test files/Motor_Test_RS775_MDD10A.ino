/*
 * ============================================================
 *  RS-775 + MDD10A 모터 드라이버 테스트 코드
 *  목표 속도: 0.4 m/s
 * ============================================================
 *  [MDD10A Sign-magnitude 모드]
 *  DIR  핀: HIGH = 전진, LOW = 후진
 *  PWM  핀: 0~255 (속도 제어)
 *
 *  핀 연결:
 *    MDD10A PWM1 → Pin 3  (모터 1 속도)
 *    MDD10A DIR1 → Pin 4  (모터 1 방향)
 *    MDD10A PWM2 → Pin 9  (모터 2 속도)
 *    MDD10A DIR2 → Pin 10 (모터 2 방향)
 *    MDD10A GND  → Arduino GND (공통 GND 필수)
 *    MDD10A VCC  → 5V
 *
 *  [속도 조정 방법]
 *  PWM_TARGET 값을 올리거나 내리면서 실측 속도 확인
 *  실측값이 0.4m/s보다 느리면 값을 올리고, 빠르면 내릴 것
 *  RS-775 사양에 따라 다르므로 반드시 실측 필요
 * ============================================================
 */

// ── 핀 설정 ─────────────────────────────────────────────────
const uint8_t PWM1_PIN = 3;   // 모터 1 속도 (PWM 핀)
const uint8_t DIR1_PIN = 4;   // 모터 1 방향
const uint8_t PWM2_PIN = 9;   // 모터 2 속도 (PWM 핀)
const uint8_t DIR2_PIN = 10;  // 모터 2 방향
// ────────────────────────────────────────────────────────────

// ── 속도 설정 ────────────────────────────────────────────────
// 0~255 범위. 실측 후 조정 필요
// 바퀴 65mm 기준 0.4m/s = 약 118RPM 목표
// RS-775 무부하 RPM과 실제 부하 RPM 차이가 크므로 실측 필수
int PWM_TARGET = 150;  // ← 이 값을 조정해서 0.4m/s 맞출 것
// ────────────────────────────────────────────────────────────

// ── 모터 제어 함수 ───────────────────────────────────────────
void setMotor(uint8_t pwmPin, uint8_t dirPin, int speed) {
  // speed: -255(최대 후진) ~ 0(정지) ~ 255(최대 전진)
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

void forward(int speed) {
  setMotor(PWM1_PIN, DIR1_PIN,  speed);
  setMotor(PWM2_PIN, DIR2_PIN,  speed);
}

void backward(int speed) {
  setMotor(PWM1_PIN, DIR1_PIN, -speed);
  setMotor(PWM2_PIN, DIR2_PIN, -speed);
}

void stopMotors() {
  setMotor(PWM1_PIN, DIR1_PIN, 0);
  setMotor(PWM2_PIN, DIR2_PIN, 0);
}

// ── 시리얼 명령어 처리 ───────────────────────────────────────
// 시리얼 모니터에서 다음 명령어로 실시간 제어 가능
// f     : 전진 (PWM_TARGET 속도)
// b     : 후진 (PWM_TARGET 속도)
// s     : 정지
// + / - : PWM 10 증가 / 감소
// p     : 현재 PWM 값 출력
void handleSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  switch (cmd) {
    case 'f':
      forward(PWM_TARGET);
      Serial.print("[전진] PWM="); Serial.println(PWM_TARGET);
      break;
    case 'b':
      backward(PWM_TARGET);
      Serial.print("[후진] PWM="); Serial.println(PWM_TARGET);
      break;
    case 's':
      stopMotors();
      Serial.println("[정지]");
      break;
    case '+':
      PWM_TARGET = constrain(PWM_TARGET + 10, 0, 255);
      Serial.print("[PWM 증가] PWM="); Serial.println(PWM_TARGET);
      forward(PWM_TARGET);
      break;
    case '-':
      PWM_TARGET = constrain(PWM_TARGET - 10, 0, 255);
      Serial.print("[PWM 감소] PWM="); Serial.println(PWM_TARGET);
      forward(PWM_TARGET);
      break;
    case 'p':
      Serial.print("[현재 PWM]="); Serial.println(PWM_TARGET);
      break;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(PWM1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);

  stopMotors();

  Serial.println("=== RS-775 + MDD10A 모터 테스트 ===");
  Serial.println("명령어: f=전진 / b=후진 / s=정지 / +=PWM+10 / -=PWM-10 / p=현재PWM");
  Serial.print("초기 PWM 설정값: "); Serial.println(PWM_TARGET);
  Serial.println("0.4m/s 목표: 실측 후 PWM_TARGET 조정 필요");
}

void loop() {
  // ── 기본 테스트 시퀀스 (시작 시 1회 실행) ─────────────────
  static bool firstRun = true;
  if (firstRun) {
    firstRun = false;

    Serial.println("\n[테스트 시퀀스 시작]");

    // 1. 전진 3초
    Serial.println("전진 3초...");
    forward(PWM_TARGET);
    delay(3000);

    // 2. 정지 1초
    Serial.println("정지 1초...");
    stopMotors();
    delay(1000);

    // 3. 후진 3초
    Serial.println("후진 3초...");
    backward(PWM_TARGET);
    delay(3000);

    // 4. 정지
    stopMotors();
    Serial.println("[테스트 시퀀스 완료]");
    Serial.println("이후 시리얼 명령어로 수동 제어 가능");
  }

  // ── 시리얼 명령어 수신 ──────────────────────────────────────
  handleSerial();
}
