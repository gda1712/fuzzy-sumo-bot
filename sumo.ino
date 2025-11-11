#include "FuzzySumoController.h"

#include <math.h>

namespace {

constexpr uint8_t IR_LEFT_PIN = A5;
constexpr uint8_t IR_CENTER_PIN = A6;
constexpr uint8_t IR_RIGHT_PIN = A7;

constexpr uint8_t ULTRA_LEFT_TRIGGER_PIN = 13;
constexpr uint8_t ULTRA_LEFT_ECHO_PIN = 14;
constexpr uint8_t ULTRA_FRONT_TRIGGER_PIN = 15;
constexpr uint8_t ULTRA_FRONT_ECHO_PIN = 16;
constexpr uint8_t ULTRA_RIGHT_TRIGGER_PIN = 17;
constexpr uint8_t ULTRA_RIGHT_ECHO_PIN = 18;

constexpr uint8_t MOTOR_LEFT_PWM_PIN = 3;
constexpr uint8_t MOTOR_LEFT_IN1_PIN = 4;
constexpr uint8_t MOTOR_LEFT_IN2_PIN = 5;

constexpr uint8_t MOTOR_RIGHT_PWM_PIN = 11;
constexpr uint8_t MOTOR_RIGHT_IN1_PIN = 6;
constexpr uint8_t MOTOR_RIGHT_IN2_PIN = 7;

constexpr uint8_t BUTTON_PIN = 9;

constexpr uint8_t kIrSensorCount = 3;
constexpr uint32_t kDebounceMs = 50U;
constexpr uint32_t kCountdownMs = 5000U;
constexpr uint32_t kUltrasonicTimeoutUs = 25000U;
constexpr float kUltrasonicMaxDistanceCm = 350.0f;
constexpr uint32_t kCalibrationCaptureMs = 5000U;
constexpr bool kButtonActiveLow = true;

enum class State : uint8_t {
  CALIBRATING_WHITE = 0,
  CALIBRATING_BLACK,
  READY_TO_START,
  IN_COMBAT,
  STOPPED
};

template <typename T>
T clampValue(T value, T lower, T upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

struct CalibrationAccumulator {
  uint32_t sum[kIrSensorCount];
  uint32_t samples;
};

const uint8_t kIrPins[kIrSensorCount] = {IR_LEFT_PIN, IR_CENTER_PIN, IR_RIGHT_PIN};

const MotorController::MotorPins kLeftMotorPins{MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_IN1_PIN, MOTOR_LEFT_IN2_PIN};
const MotorController::MotorPins kRightMotorPins{MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_IN1_PIN, MOTOR_RIGHT_IN2_PIN};

MotorController motorController(kLeftMotorPins, kRightMotorPins);
FuzzySumoController fuzzyController;
FuzzySumoController::Config fuzzyConfig = FuzzySumoController::Config::makeDefault();

State currentState = State::CALIBRATING_WHITE;
CalibrationAccumulator whiteCalibration{};
CalibrationAccumulator blackCalibration{};
uint16_t irMinValues[kIrSensorCount] = {1023U, 1023U, 1023U};
uint16_t irMaxValues[kIrSensorCount] = {0U, 0U, 0U};

bool calibrationWindowActive = false;
uint32_t calibrationStartMs = 0U;

bool countdownActive = false;
uint32_t countdownStartMs = 0U;

bool lastButtonReading = false;
bool stableButtonState = false;
bool buttonPressedEvent = false;
uint32_t lastDebounceMs = 0U;

void resetCalibrationAccumulator(CalibrationAccumulator &acc) {
  for (uint8_t i = 0; i < kIrSensorCount; ++i) {
    acc.sum[i] = 0U;
  }
  acc.samples = 0U;
}

int8_t irPinToIndex(uint8_t pin) {
  for (uint8_t i = 0; i < kIrSensorCount; ++i) {
    if (kIrPins[i] == pin) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

void enterState(State newState) {
  currentState = newState;

  switch (currentState) {
    case State::CALIBRATING_WHITE:
      resetCalibrationAccumulator(whiteCalibration);
      calibrationWindowActive = false;
      motorController.stop();
      break;
    case State::CALIBRATING_BLACK:
      resetCalibrationAccumulator(blackCalibration);
      calibrationWindowActive = false;
      motorController.stop();
      break;
    case State::READY_TO_START:
      countdownActive = false;
      calibrationWindowActive = false;
      motorController.stop();
      break;
    case State::IN_COMBAT:
      countdownActive = false;
      calibrationWindowActive = false;
      break;
    case State::STOPPED:
      calibrationWindowActive = false;
      motorController.stop();
      break;
  }
}

void calibrateIRSensors() {
  CalibrationAccumulator *target = nullptr;
  if (currentState == State::CALIBRATING_WHITE) {
    target = &whiteCalibration;
  } else if (currentState == State::CALIBRATING_BLACK) {
    target = &blackCalibration;
  } else {
    return;
  }

  for (uint8_t i = 0; i < kIrSensorCount; ++i) {
    const uint16_t reading = analogRead(kIrPins[i]);
    target->sum[i] += reading;
  }
  ++target->samples;
}

void finalizeWhiteCalibration() {
  if (whiteCalibration.samples == 0U) {
    for (uint8_t i = 0; i < kIrSensorCount; ++i) {
      irMinValues[i] = analogRead(kIrPins[i]);
    }
    return;
  }

  for (uint8_t i = 0; i < kIrSensorCount; ++i) {
    const uint32_t average = whiteCalibration.sum[i] / whiteCalibration.samples;
    irMinValues[i] = static_cast<uint16_t>(clampValue<uint32_t>(average, 0U, 1023U));
  }
}

void finalizeBlackCalibration() {
  if (blackCalibration.samples == 0U) {
    for (uint8_t i = 0; i < kIrSensorCount; ++i) {
      const uint16_t reading = analogRead(kIrPins[i]);
      irMaxValues[i] = static_cast<uint16_t>(clampValue<uint16_t>(reading, irMinValues[i] + 1U, 1023U));
    }
  } else {
    for (uint8_t i = 0; i < kIrSensorCount; ++i) {
      const uint32_t average = blackCalibration.sum[i] / blackCalibration.samples;
      irMaxValues[i] = static_cast<uint16_t>(clampValue<uint32_t>(average, 0U, 1023U));
      if (irMaxValues[i] <= irMinValues[i]) {
        irMaxValues[i] = clampValue<uint16_t>(irMinValues[i] + 10U, 0U, 1023U);
      }
    }
  }
}

int readNormalizedIR(uint8_t pin) {
  const int8_t index = irPinToIndex(pin);
  if (index < 0) {
    return 0;
  }

  const uint16_t minVal = irMinValues[index];
  const uint16_t maxVal = irMaxValues[index];
  const uint16_t raw = analogRead(pin);

  if (maxVal <= minVal + 1U) {
    return 0;
  }

  const uint16_t clipped = clampValue<uint16_t>(raw, minVal, maxVal);
  const uint32_t span = static_cast<uint32_t>(maxVal - minVal);
  const uint32_t value = static_cast<uint32_t>(clipped - minVal);
  const uint32_t normalized = (value * 1000UL) / span;

  return static_cast<int>(clampValue<uint32_t>(normalized, 0U, 1000U));
}

float getUltrasonicDistance(uint8_t triggerPin, uint8_t echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, kUltrasonicTimeoutUs);
  if (duration == 0UL) {
    return kUltrasonicMaxDistanceCm;
  }

  const float distance = (static_cast<float>(duration) * 0.0343f) * 0.5f;
  return clampValue(distance, 0.0f, kUltrasonicMaxDistanceCm);
}

void updateButton(uint32_t nowMs) {
  bool reading = (digitalRead(BUTTON_PIN) != 0);
  if (kButtonActiveLow) {
    reading = !reading;
  }

  if (reading != lastButtonReading) {
    lastDebounceMs = nowMs;
    lastButtonReading = reading;
  }

  if ((nowMs - lastDebounceMs) >= kDebounceMs && reading != stableButtonState) {
    stableButtonState = reading;
    if (stableButtonState) {
      buttonPressedEvent = true;
    }
  }
}

bool consumeButtonPress() {
  if (!buttonPressedEvent) {
    return false;
  }
  buttonPressedEvent = false;
  return true;
}

void runCombatIteration() {
  FuzzySumoController::Input input{};
  input.leftDistanceCm = getUltrasonicDistance(ULTRA_LEFT_TRIGGER_PIN, ULTRA_LEFT_ECHO_PIN);
  input.frontDistanceCm = getUltrasonicDistance(ULTRA_FRONT_TRIGGER_PIN, ULTRA_FRONT_ECHO_PIN);
  input.rightDistanceCm = getUltrasonicDistance(ULTRA_RIGHT_TRIGGER_PIN, ULTRA_RIGHT_ECHO_PIN);

  input.leftEdgeNorm = static_cast<float>(readNormalizedIR(IR_LEFT_PIN));
  input.centerEdgeNorm = static_cast<float>(readNormalizedIR(IR_CENTER_PIN));
  input.rightEdgeNorm = static_cast<float>(readNormalizedIR(IR_RIGHT_PIN));

  const FuzzySumoController::Output output = fuzzyController.evaluate(input);

  const int leftSpeed = static_cast<int>(lroundf(output.leftMotorSpeed));
  const int rightSpeed = static_cast<int>(lroundf(output.rightMotorSpeed));
  motorController.move(leftSpeed, rightSpeed);
}

}  // namespace

void setup() {
  for (uint8_t i = 0; i < kIrSensorCount; ++i) {
    pinMode(kIrPins[i], INPUT);
  }

  pinMode(ULTRA_LEFT_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRA_FRONT_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRA_RIGHT_TRIGGER_PIN, OUTPUT);
  digitalWrite(ULTRA_LEFT_TRIGGER_PIN, LOW);
  digitalWrite(ULTRA_FRONT_TRIGGER_PIN, LOW);
  digitalWrite(ULTRA_RIGHT_TRIGGER_PIN, LOW);

  pinMode(ULTRA_LEFT_ECHO_PIN, INPUT);
  pinMode(ULTRA_FRONT_ECHO_PIN, INPUT);
  pinMode(ULTRA_RIGHT_ECHO_PIN, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  motorController.begin();
  motorController.stop();

  const bool pressed = kButtonActiveLow ? (digitalRead(BUTTON_PIN) == LOW) : (digitalRead(BUTTON_PIN) == HIGH);
  lastButtonReading = pressed;
  stableButtonState = pressed;
  buttonPressedEvent = false;
  lastDebounceMs = millis();

  resetCalibrationAccumulator(whiteCalibration);
  resetCalibrationAccumulator(blackCalibration);
  calibrationWindowActive = false;
  calibrationStartMs = 0U;
  fuzzyController.begin(fuzzyConfig);
  enterState(State::CALIBRATING_WHITE);
}

void loop() {
  const uint32_t nowMs = millis();
  updateButton(nowMs);

  switch (currentState) {
    case State::CALIBRATING_WHITE:
      if (calibrationWindowActive) {
        calibrateIRSensors();
        if ((nowMs - calibrationStartMs) >= kCalibrationCaptureMs) {
          finalizeWhiteCalibration();
          calibrationWindowActive = false;
          enterState(State::CALIBRATING_BLACK);
        }
      } else if (consumeButtonPress()) {
        resetCalibrationAccumulator(whiteCalibration);
        calibrationWindowActive = true;
        calibrationStartMs = nowMs;
      }
      break;

    case State::CALIBRATING_BLACK:
      if (calibrationWindowActive) {
        calibrateIRSensors();
        if ((nowMs - calibrationStartMs) >= kCalibrationCaptureMs) {
          finalizeBlackCalibration();
          calibrationWindowActive = false;
          enterState(State::READY_TO_START);
        }
      } else if (consumeButtonPress()) {
        resetCalibrationAccumulator(blackCalibration);
        calibrationWindowActive = true;
        calibrationStartMs = nowMs;
      }
      break;

    case State::READY_TO_START:
      motorController.stop();
      if (countdownActive) {
        if ((nowMs - countdownStartMs) >= kCountdownMs) {
          countdownActive = false;
          enterState(State::IN_COMBAT);
        }
      } else if (consumeButtonPress()) {
        countdownActive = true;
        countdownStartMs = nowMs;
      }
      break;

    case State::IN_COMBAT:
      if (consumeButtonPress()) {
        motorController.stop();
        enterState(State::STOPPED);
        break;
      }
      runCombatIteration();
      break;

    case State::STOPPED:
      motorController.stop();
      if (consumeButtonPress()) {
        enterState(State::READY_TO_START);
      }
      break;
  }
}

