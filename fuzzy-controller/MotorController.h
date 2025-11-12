#pragma once

#include <Arduino.h>

class MotorController {
public:
  struct MotorPins {
    uint8_t pwmPin;
    uint8_t in1Pin;
    uint8_t in2Pin;
  };

  MotorController(const MotorPins &leftPins, const MotorPins &rightPins, uint8_t maxPwm = 255);

  void begin() const;
  void move(int leftSpeed, int rightSpeed) const;
  void stop() const;

private:
  MotorPins leftPins_;
  MotorPins rightPins_;
  uint8_t maxPwm_;
};


