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

Serial.println("Applying motor pins");
Serial.println(pins.pwmPin);
Serial.println(clampValue(duty, 0, static_cast<int>(maxPwm)));
Serial.println(speed);
    digitalWrite(pins.in1Pin, LOW);
    digitalWrite(pins.in2Pin, HIGH);
  

  analogWrite(pins.pwmPin, clampValue(duty, 0, static_cast<int>(maxPwm)));
}

}  // namespace

MotorController::MotorController(const MotorPins &leftPins, const MotorPins &rightPins, uint8_t standbyPin,
                                 uint8_t maxPwm)
    : leftPins_(leftPins), rightPins_(rightPins), maxPwm_(maxPwm) {}

void MotorController::begin() const {
  pinMode(leftPins_.pwmPin, OUTPUT);
  pinMode(leftPins_.in1Pin, OUTPUT);
  pinMode(leftPins_.in2Pin, OUTPUT);

  pinMode(rightPins_.pwmPin, OUTPUT);
  pinMode(rightPins_.in1Pin, OUTPUT);
  pinMode(rightPins_.in2Pin, OUTPUT);

  stop();
}

void MotorController::move(int leftSpeed, int rightSpeed) const {

  applyMotorPins(leftPins_, leftSpeed, maxPwm_);
  applyMotorPins(rightPins_, rightSpeed, maxPwm_);
}

void MotorController::stop() const {
  applyMotorPins(leftPins_, 0, maxPwm_);
  applyMotorPins(rightPins_, 0, maxPwm_);

  
}


