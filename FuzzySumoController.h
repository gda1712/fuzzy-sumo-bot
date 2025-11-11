#pragma once

#include <Arduino.h>
#include <qlibs.h>

class MotorController {
public:
  struct MotorPins {
    uint8_t pwmPin;
    uint8_t in1Pin;
    uint8_t in2Pin;
  };

  MotorController(const MotorPins &leftPins, const MotorPins &rightPins, uint8_t standbyPin = 0xFF,
                  uint8_t maxPwm = 255);

  void begin() const;
  void move(int leftSpeed, int rightSpeed) const;
  void stop() const;

private:
  MotorPins leftPins_;
  MotorPins rightPins_;
  uint8_t standbyPin_;
  uint8_t maxPwm_;
};

class FuzzySumoController {
public:
  using real_t = qlibs::real_t;

  static constexpr size_t kInputCount = 6U;
  static constexpr size_t kOutputCount = 2U;
  static constexpr size_t kDistanceMfPerInput = 3U;
  static constexpr size_t kEdgeMfPerInput = 2U;
  static constexpr size_t kMotorMfPerOutput = 5U;
  static constexpr size_t kInputMfCount =
      3U * kDistanceMfPerInput + 3U * kEdgeMfPerInput;  // 3 distance sensors + 3 edge sensors
  static constexpr size_t kOutputMfCount = kOutputCount * kMotorMfPerOutput;
  static constexpr size_t kRuleCount = 7U;

  struct Input {
    real_t leftDistanceCm = 0.0f;
    real_t frontDistanceCm = 0.0f;
    real_t rightDistanceCm = 0.0f;
    real_t leftEdgeNorm = 0.0f;
    real_t centerEdgeNorm = 0.0f;
    real_t rightEdgeNorm = 0.0f;
  };

  struct Output {
    real_t leftMotorSpeed = 0.0f;
    real_t rightMotorSpeed = 0.0f;
  };

  struct Triangle {
    real_t points[3];
    constexpr const real_t *data() const { return points; }
  };

  struct Trapezoid {
    real_t points[4];
    constexpr const real_t *data() const { return points; }
  };

  struct Config {
    real_t distanceRange[2];
    real_t edgeRange[2];
    real_t motorRange[2];

    struct DistanceSets {
      Triangle close;
      Triangle medium;
      Trapezoid far;
    };

    struct EdgeSets {
      Trapezoid safe;
      Trapezoid danger;
    };

    struct MotorSets {
      Trapezoid fastReverse;
      Triangle slowReverse;
      Triangle stop;
      Triangle slowForward;
      Trapezoid fastForward;
    };

    DistanceSets leftDistance;
    DistanceSets frontDistance;
    DistanceSets rightDistance;

    EdgeSets leftEdge;
    EdgeSets centerEdge;
    EdgeSets rightEdge;

    MotorSets leftMotor;
    MotorSets rightMotor;

    qlibs::fis::deFuzzMethod defuzzMethod = qlibs::fis::centroid;

    static Config makeDefault();
  };

  FuzzySumoController();

  void setConfig(const Config &config);
  void begin(const Config &config);
  Output evaluate(const Input &input);
  const Config &config() const { return config_; }

private:
  void initialize();

  template <typename T>
  static T clampValue(T value, T minValue, T maxValue) {
    if (value < minValue) {
      return minValue;
    }
    if (value > maxValue) {
      return maxValue;
    }
    return value;
  }

  Config config_{Config::makeDefault()};
  bool initialized_ = false;

  qlibs::fis::instance controller_{};
  qlibs::fis::input inputs_[kInputCount]{};
  qlibs::fis::output outputs_[kOutputCount]{};
  qlibs::fis::mf inputMF_[kInputMfCount]{};
  qlibs::fis::mf outputMF_[kOutputMfCount]{};
  qlibs::real_t ruleStrength_[kRuleCount]{};
};

