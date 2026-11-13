/*
  ESP32-CAM + ULN2003 + 28BYJ-48 (5 V) quick test

  Wiring (ULN2003 -> ESP32-CAM):
    IN1 -> GPIO 4
    IN2 -> GPIO 2
    IN3 -> GPIO 14
    IN4 -> GPIO 15

  Power:
    - Power the 28BYJ-48/ULN2003 from a stable external 5 V supply.
    - Connect that supply's GND to ESP32-CAM GND.
    - Do NOT power the motor from the USB-to-serial programmer's 5 V pin.

  Note: GPIO4 also controls the ESP32-CAM's onboard flash LED, so the LED may
  flicker during the test. GPIO2 and GPIO15 are boot-strapping pins. If upload
  fails with the driver connected, temporarily remove ULN2003 power (or the
  IN leads) while entering download mode, then reconnect them after upload.
*/

constexpr uint8_t IN1_PIN = 4;
constexpr uint8_t IN2_PIN = 2;
constexpr uint8_t IN3_PIN = 14;
constexpr uint8_t IN4_PIN = 15;

constexpr uint16_t STEP_DELAY_MS = 2;     // Smaller = faster; begin at 2 ms.
constexpr uint16_t TEST_STEPS = 512;      // About 1/8 output-shaft revolution.
constexpr uint16_t PAUSE_MS = 800;

// 8-state half-step sequence: smoothest mode for a 28BYJ-48.
constexpr uint8_t HALF_STEP[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1},
};

int8_t phase = 0;

void writePhase(uint8_t index) {
  digitalWrite(IN1_PIN, HALF_STEP[index][0]);
  digitalWrite(IN2_PIN, HALF_STEP[index][1]);
  digitalWrite(IN3_PIN, HALF_STEP[index][2]);
  digitalWrite(IN4_PIN, HALF_STEP[index][3]);
}

void releaseMotor() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void moveSteps(int steps) {
  const int direction = (steps >= 0) ? 1 : -1;
  const uint16_t count = abs(steps);

  for (uint16_t i = 0; i < count; ++i) {
    phase = (phase + direction + 8) % 8;
    writePhase(phase);
    delay(STEP_DELAY_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  releaseMotor();

  Serial.println("ESP32-CAM 28BYJ-48 quick test started");
}

void loop() {
  Serial.println("Forward");
  moveSteps(TEST_STEPS);
  releaseMotor();
  delay(PAUSE_MS);

  Serial.println("Reverse");
  moveSteps(-TEST_STEPS);
  releaseMotor();
  delay(PAUSE_MS);
}
