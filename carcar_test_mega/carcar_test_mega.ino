// Arduino Mega 2560 test bench: IR array + RC522 + TB6612 motors.
// Wiring: IR A3..A7, RC522 SS=2 RST=3 (SPI 50/51/52), TB6612 PWMA=10 AIN1=7 AIN2=6
// PWMB=11 BIN1=8 BIN2=9 (STBY tied to 5V), TX1 (D18) -> ESP32 GPIO16 (RX2).

#include <SPI.h>
#include <MFRC522.h>

const uint8_t IR_PINS[5] = {A3, A4, A5, A6, A7};
const uint8_t RC522_SS = 2;
const uint8_t RC522_RST = 3;
const int TEST_PWM = 150;
const unsigned long TICK_MS = 100;
const uint8_t UID_MAX_BYTES = 10;

struct Motor {
  uint8_t pwmPin;
  uint8_t in1;
  uint8_t in2;
};

struct Phase {
  int pwmA;
  int pwmB;
  unsigned long durationMs;
};

const Motor MOTOR_A = {10, 7, 6};
const Motor MOTOR_B = {11, 8, 9};

const Phase PHASES[] = {
  {0, 0, 3000},
  {TEST_PWM, 0, 3000},   {0, 0, 1000},
  {-TEST_PWM, 0, 3000},  {0, 0, 1000},
  {0, TEST_PWM, 3000},   {0, 0, 1000},
  {0, -TEST_PWM, 3000},  {0, 0, 1000},
  {TEST_PWM, TEST_PWM, 3000},   {0, 0, 1000},
  {-TEST_PWM, -TEST_PWM, 3000}, {0, 0, 1000},
};
const size_t PHASE_COUNT = sizeof(PHASES) / sizeof(PHASES[0]);

MFRC522 rfid(RC522_SS, RC522_RST);

size_t phaseIndex = 0;
unsigned long phaseStartMs = 0;
unsigned long lastTickMs = 0;

char lastUid[2 * UID_MAX_BYTES + 1] = "-";
bool readerOk = true;
unsigned long scans = 0;

void setMotor(const Motor& m, int signedPwm) {
  signedPwm = constrain(signedPwm, -255, 255);
  digitalWrite(m.in1, signedPwm > 0 ? HIGH : LOW);
  digitalWrite(m.in2, signedPwm < 0 ? HIGH : LOW);
  analogWrite(m.pwmPin, abs(signedPwm));
}

void applyPhase(size_t i) {
  setMotor(MOTOR_A, PHASES[i].pwmA);
  setMotor(MOTOR_B, PHASES[i].pwmB);
}

void initMotor(const Motor& m) {
  pinMode(m.pwmPin, OUTPUT);
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
}

void updateRfid() {
  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  bool present = version != 0x00 && version != 0xFF;
  if (present && !readerOk) {
    rfid.PCD_Init();  // the chip may have lost power while it was missing
  }
  readerOk = present;
  if (!readerOk) return;

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    uint8_t n = min(rfid.uid.size, UID_MAX_BYTES);
    for (uint8_t i = 0; i < n; i++) {
      snprintf(lastUid + 2 * i, 3, "%02X", rfid.uid.uidByte[i]);
    }
    scans++;
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }
}

void sendTelemetry() {
  char line[96];
  snprintf(line, sizeof(line), "T,%d,%d,%d,%d,%d,%s,%lu,%d,%d\n",
           analogRead(IR_PINS[0]), analogRead(IR_PINS[1]), analogRead(IR_PINS[2]),
           analogRead(IR_PINS[3]), analogRead(IR_PINS[4]),
           readerOk ? lastUid : "!", scans,
           PHASES[phaseIndex].pwmA, PHASES[phaseIndex].pwmB);
  Serial1.print(line);
  Serial.print(line);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  initMotor(MOTOR_A);
  initMotor(MOTOR_B);
  SPI.begin();
  rfid.PCD_Init();
  applyPhase(0);
  phaseStartMs = lastTickMs = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - phaseStartMs >= PHASES[phaseIndex].durationMs) {
    phaseIndex = (phaseIndex + 1) % PHASE_COUNT;
    phaseStartMs = now;
    applyPhase(phaseIndex);
  }

  if (now - lastTickMs >= TICK_MS) {
    lastTickMs += TICK_MS;
    updateRfid();
    sendTelemetry();
  }
}
