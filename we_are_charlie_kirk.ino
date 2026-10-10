/*
  ESP32-S3 MEDKIT-ONLY CONTROLLER

  Required libraries:
    - ESP32Servo
    - Adafruit VL53L0X

  Motor wiring:
    Left:  PWM GPIO4, direction GPIO5/GPIO6
    Right: PWM GPIO7, direction GPIO8/GPIO9

  Line sensors:
    S2 GPIO1, S3 GPIO2, S4 GPIO3
    HIGH = black

  VL53L0X:
    SDA GPIO18, SCL GPIO21

  Servos:
    SV5 Yellow 1/right: GPIO38
    SV6 Yellow 2/middle: GPIO39
    SV7 Red/left: GPIO40

  Use an external regulated servo supply.
  Connect all grounds together.
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_VL53L0X.h>

// ---------------- Pins ----------------

const int LEFT_PWM  = 4;
const int LEFT_DIR1 = 5;
const int LEFT_DIR2 = 6;

const int RIGHT_PWM  = 7;
const int RIGHT_DIR1 = 8;
const int RIGHT_DIR2 = 9;

const int ENC_LF_C1 = 10;
const int ENC_LR_C1 = 12;
const int ENC_RF_C1 = 14;
const int ENC_RR_C1 = 16;

const int IR_S2 = 1;
const int IR_S3 = 2;
const int IR_S4 = 3;

const int I2C_SDA = 18;
const int I2C_SCL = 21;
const uint8_t TOF_ADDRESS = 0x29;

const int SV5_PIN = 38;
const int SV6_PIN = 39;
const int SV7_PIN = 40;

// ---------------- Calibration ----------------

const bool BLACK_IS_HIGH = true;

const uint32_t LINE_DEBOUNCE_MS = 80;
const uint32_t REVERSE_1_MS = 500;
const uint32_t REVERSE_2_MS = 1000;
const uint32_t TURN_90_ARC_MS = 1000;  // Calibrate
const uint32_t AFTER_LINE_MS = 1000;
const uint32_t RED_TRAVEL_MAX_MS = 3000;
const uint32_t AFTER_TOF_MS = 1000;

const uint8_t DRIVE_PWM = 150;
const uint8_t REVERSE_PWM = 130;
const uint8_t TURN_INNER_PWM = 75;
const uint8_t TURN_OUTER_PWM = 180;

const float TOF_CRITICAL_MM = 58.0f;
const uint8_t TOF_CONFIRM_COUNT = 3;

const int SV_CLOSED = 20;
const int SV5_OPEN = 105;
const int SV6_OPEN = 105;
const int SV7_OPEN = 105;

const uint32_t SERVO_HOLD_MS = 600;

// ---------------- State definitions ----------------

enum State {
  START_FORWARD,
  WAIT_SECOND_BLACK,
  REVERSE_BEFORE_TURN,
  ROLLING_LEFT_90,
  REVERSE_IN_PCC,
  RELEASE_SV5,
  WAIT_PCC_BLACK,
  AFTER_PCC_LINE,
  RELEASE_SV7,
  RED_TRAVEL,
  AFTER_TOF,
  RELEASE_SV6,
  FINISHED,
  FAULT
};

// Explicit declaration fixes Arduino prototype generation.
void enterState(State next);

State state = START_FORWARD;
uint32_t stateStarted = 0;

// ---------------- Peripherals ----------------

Servo sv5;
Servo sv6;
Servo sv7;

Adafruit_VL53L0X tof;

volatile int32_t encoderLF = 0;
volatile int32_t encoderLR = 0;
volatile int32_t encoderRF = 0;
volatile int32_t encoderRR = 0;

void IRAM_ATTR isrLF() { encoderLF++; }
void IRAM_ATTR isrLR() { encoderLR++; }
void IRAM_ATTR isrRF() { encoderRF++; }
void IRAM_ATTR isrRR() { encoderRR++; }

// ---------------- Motors ----------------

void setOneMotor(int pwmPin, int dir1, int dir2, int signedPwm) {
  signedPwm = constrain(signedPwm, -255, 255);

  if (signedPwm > 0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(pwmPin, signedPwm);
  } else if (signedPwm < 0) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwmPin, -signedPwm);
  } else {
    analogWrite(pwmPin, 0);
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
  }
}

void driveSides(int leftPwm, int rightPwm) {
  setOneMotor(LEFT_PWM, LEFT_DIR1, LEFT_DIR2, leftPwm);
  setOneMotor(RIGHT_PWM, RIGHT_DIR1, RIGHT_DIR2, rightPwm);
}

void stopMotors() {
  driveSides(0, 0);
}

void forward() {
  driveSides(DRIVE_PWM, DRIVE_PWM);
}

void reverseRobot() {
  driveSides(-REVERSE_PWM, -REVERSE_PWM);
}

void rollingLeftTurn() {
  // Both sides forward; right side faster.
  driveSides(TURN_INNER_PWM, TURN_OUTER_PWM);
}

// ---------------- Sensors ----------------

bool allBlackRaw() {
  const int blackLevel = BLACK_IS_HIGH ? HIGH : LOW;

  return digitalRead(IR_S2) == blackLevel &&
         digitalRead(IR_S3) == blackLevel &&
         digitalRead(IR_S4) == blackLevel;
}

bool allBlackConfirmed() {
  static bool last = false;
  static uint32_t changedAt = 0;

  const bool now = allBlackRaw();

  if (now != last) {
    last = now;
    changedAt = millis();
  }

  return now && millis() - changedAt >= LINE_DEBOUNCE_MS;
}

bool tofCritical() {
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);

  if (measure.RangeStatus != 0) return false;
  if (measure.RangeMilliMeter < 30 ||
      measure.RangeMilliMeter > 2000) {
    return false;
  }

  static uint8_t closeCount = 0;

  if (measure.RangeMilliMeter <= TOF_CRITICAL_MM) {
    if (closeCount < TOF_CONFIRM_COUNT) {
      closeCount++;
    }
  } else {
    closeCount = 0;
  }

  return closeCount >= TOF_CONFIRM_COUNT;
}

void waitUntilWhite() {
  const uint32_t start = millis();

  while (allBlackConfirmed() && millis() - start < 3000) {
    forward();
    delay(10);
  }
}

// ---------------- Servos ----------------

void initialiseServos() {
  sv5.attach(SV5_PIN);
  sv6.attach(SV6_PIN);
  sv7.attach(SV7_PIN);

  sv5.write(SV_CLOSED);
  sv6.write(SV_CLOSED);
  sv7.write(SV_CLOSED);

  delay(300);
}

void releaseServo(Servo &servo, int openAngle, const char *name) {
  stopMotors();

  Serial.print("RELEASE ");
  Serial.println(name);

  servo.write(openAngle);
  delay(SERVO_HOLD_MS);

  servo.write(SV_CLOSED);
  delay(250);
}

// ---------------- State control ----------------

void enterState(State next) {
  state = next;
  stateStarted = millis();

  Serial.print("STATE ");
  Serial.println(static_cast<int>(state));
}

// ---------------- Setup ----------------

void setup() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR1, OUTPUT);
  pinMode(LEFT_DIR2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR1, OUTPUT);
  pinMode(RIGHT_DIR2, OUTPUT);

  digitalWrite(LEFT_PWM, LOW);
  digitalWrite(RIGHT_PWM, LOW);
  digitalWrite(LEFT_DIR1, LOW);
  digitalWrite(LEFT_DIR2, LOW);
  digitalWrite(RIGHT_DIR1, LOW);
  digitalWrite(RIGHT_DIR2, LOW);

  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32-S3 medkit-only controller starting");

  pinMode(IR_S2, INPUT);
  pinMode(IR_S3, INPUT);
  pinMode(IR_S4, INPUT);

  pinMode(ENC_LF_C1, INPUT_PULLUP);
  pinMode(ENC_LR_C1, INPUT_PULLUP);
  pinMode(ENC_RF_C1, INPUT_PULLUP);
  pinMode(ENC_RR_C1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LF_C1), isrLF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LR_C1), isrLR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RF_C1), isrRF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_C1), isrRR, RISING);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!tof.begin(TOF_ADDRESS, false, &Wire)) {
    Serial.println("FAULT: VL53L0X not found");
    state = FAULT;
  }

  initialiseServos();
  stopMotors();

  stateStarted = millis();
}

// ---------------- Main loop ----------------

void loop() {
  switch (state) {
    case START_FORWARD:
      forward();

      if (allBlackConfirmed()) {
        Serial.println("First black line: ignored");
        enterState(WAIT_SECOND_BLACK);
        waitUntilWhite();
      }
      break;

    case WAIT_SECOND_BLACK:
      forward();

      if (allBlackConfirmed()) {
        Serial.println("Second black line: route marker");
        enterState(REVERSE_BEFORE_TURN);
      }
      break;

    case REVERSE_BEFORE_TURN:
      reverseRobot();

      if (millis() - stateStarted >= REVERSE_1_MS) {
        enterState(ROLLING_LEFT_90);
      }
      break;

    case ROLLING_LEFT_90:
      rollingLeftTurn();

      if (millis() - stateStarted >= TURN_90_ARC_MS) {
        enterState(REVERSE_IN_PCC);
      }
      break;

    case REVERSE_IN_PCC:
      reverseRobot();

      if (millis() - stateStarted >= REVERSE_2_MS) {
        enterState(RELEASE_SV5);
      }
      break;

    case RELEASE_SV5:
      releaseServo(sv5, SV5_OPEN, "SV5 Yellow 1/right");
      enterState(WAIT_PCC_BLACK);
      waitUntilWhite();
      break;

    case WAIT_PCC_BLACK:
      forward();

      if (allBlackConfirmed()) {
        enterState(AFTER_PCC_LINE);
      }
      break;

    case AFTER_PCC_LINE:
      forward();

      if (millis() - stateStarted >= AFTER_LINE_MS) {
        enterState(RELEASE_SV7);
      }
      break;

    case RELEASE_SV7:
      releaseServo(sv7, SV7_OPEN, "SV7 Red/left");
      enterState(RED_TRAVEL);
      break;

    case RED_TRAVEL:
      forward();

      // Preserves the backup behavior:
      // either trigger is followed by 1 additional second.
      if (tofCritical() ||
          millis() - stateStarted >= RED_TRAVEL_MAX_MS) {
        enterState(AFTER_TOF);
      }
      break;

    case AFTER_TOF:
      forward();

      if (millis() - stateStarted >= AFTER_TOF_MS) {
        enterState(RELEASE_SV6);
      }
      break;

    case RELEASE_SV6:
      releaseServo(sv6, SV6_OPEN, "SV6 Yellow 2/middle");
      enterState(FINISHED);
      break;

    case FINISHED:
      stopMotors();
      delay(1000);
      break;

    case FAULT:
    default:
      stopMotors();
      delay(250);
      break;
  }

  delay(10);
}
