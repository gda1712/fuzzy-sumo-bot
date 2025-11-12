#include "MotorController.h"

namespace {

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

void applyMotorPins(const MotorController::MotorPins &pins, int speed, uint8_t maxPwm) {
  const int limited = clampValue(speed, -static_cast<int>(maxPwm), static_cast<int>(maxPwm));
  const bool forward = limited >= 0;
  const int duty = forward ? limited : -limited;

  if (limited == 0) {
    digitalWrite(pins.in1Pin, LOW);
    digitalWrite(pins.in2Pin, LOW);
  } else if (forward) {
    digitalWrite(pins.in1Pin, HIGH);
    digitalWrite(pins.in2Pin, LOW);
  } else {
    digitalWrite(pins.in1Pin, LOW);
    digitalWrite(pins.in2Pin, HIGH);
  }

  analogWrite(pins.pwmPin, clampValue(duty, 0, maxPwm));
}

}  // namespace

MotorController::MotorController(const MotorPins &leftPins, const MotorPins &rightPins, uint8_t standbyPin,
                                 uint8_t maxPwm)
    : leftPins_(leftPins), rightPins_(rightPins), standbyPin_(standbyPin), maxPwm_(maxPwm) {}

void MotorController::begin() const {
  pinMode(leftPins_.pwmPin, OUTPUT);
  pinMode(leftPins_.in1Pin, OUTPUT);
  pinMode(leftPins_.in2Pin, OUTPUT);

  pinMode(rightPins_.pwmPin, OUTPUT);
  pinMode(rightPins_.in1Pin, OUTPUT);
  pinMode(rightPins_.in2Pin, OUTPUT);

  if (standbyPin_ != 0xFF) {
    pinMode(standbyPin_, OUTPUT);
    digitalWrite(standbyPin_, HIGH);
  }

  stop();
}

void MotorController::move(int leftSpeed, int rightSpeed) const {
  if (standbyPin_ != 0xFF) {
    digitalWrite(standbyPin_, HIGH);
  }

  applyMotorPins(leftPins_, leftSpeed, maxPwm_);
  applyMotorPins(rightPins_, rightSpeed, maxPwm_);
}

void MotorController::stop() const {
  applyMotorPins(leftPins_, 0, maxPwm_);
  applyMotorPins(rightPins_, 0, maxPwm_);

  if (standbyPin_ != 0xFF) {
    digitalWrite(standbyPin_, LOW);
  }
}


