#include "FuzzySumoController.h"

#include <math.h>
#include <string.h>

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

enum : qlibs::fis::tag {
  in_left_distance,
  in_front_distance,
  in_right_distance,
  in_left_edge,
  in_center_edge,
  in_right_edge
};

enum : qlibs::fis::tag { out_left_motor, out_right_motor };

enum : qlibs::fis::tag {
  left_distance_close,
  left_distance_medium,
  left_distance_far,
  front_distance_close,
  front_distance_medium,
  front_distance_far,
  right_distance_close,
  right_distance_medium,
  right_distance_far,
  left_edge_safe,
  left_edge_danger,
  center_edge_safe,
  center_edge_danger,
  right_edge_safe,
  right_edge_danger,
  left_motor_fast_reverse,
  left_motor_slow_reverse,
  left_motor_stop,
  left_motor_slow_forward,
  left_motor_fast_forward,
  right_motor_fast_reverse,
  right_motor_slow_reverse,
  right_motor_stop,
  right_motor_slow_forward,
  right_motor_fast_forward
};

const qlibs::fis::rules kRules[] = {
    FIS_RULES_BEGIN
    IF in_right_edge IS right_edge_danger THEN out_left_motor IS left_motor_fast_reverse AND out_right_motor IS right_motor_slow_reverse END
    IF in_left_edge IS left_edge_danger THEN out_left_motor IS left_motor_slow_reverse AND out_right_motor IS right_motor_fast_reverse END
    IF in_center_edge IS center_edge_danger THEN out_left_motor IS left_motor_fast_reverse AND out_right_motor IS right_motor_fast_reverse END
    IF in_front_distance IS front_distance_close THEN out_left_motor IS left_motor_fast_forward AND out_right_motor IS right_motor_fast_forward END
    IF in_right_distance IS right_distance_close THEN out_left_motor IS left_motor_fast_forward AND out_right_motor IS right_motor_slow_forward END
    IF in_left_distance IS left_distance_close THEN out_left_motor IS left_motor_slow_forward AND out_right_motor IS right_motor_fast_forward END
    IF in_front_distance IS front_distance_far AND in_left_distance IS left_distance_far AND in_right_distance IS right_distance_far THEN out_left_motor IS left_motor_slow_forward AND out_right_motor IS right_motor_slow_reverse END
    FIS_RULES_END
};

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

FuzzySumoController::Config FuzzySumoController::Config::makeDefault() {
  Config cfg{};

  cfg.distanceRange[0] = 0.0f;
  cfg.distanceRange[1] = 350.0f;

  cfg.edgeRange[0] = 0.0f;
  cfg.edgeRange[1] = 1000.0f;

  cfg.motorRange[0] = -255.0f;
  cfg.motorRange[1] = 255.0f;

  // Distances in centimetres (cm)
  const auto setDistanceDefaults = [](DistanceSets &set) {
    set.close.points[0] = 0.0f;
    set.close.points[1] = 15.0f;
    set.close.points[2] = 35.0f;

    set.medium.points[0] = 25.0f;
    set.medium.points[1] = 55.0f;
    set.medium.points[2] = 85.0f;

    set.far.points[0] = 70.0f;
    set.far.points[1] = 120.0f;
    set.far.points[2] = 220.0f;
    set.far.points[3] = 320.0f;
  };

  // Edge sensor values normalised to a 0–1000 scale
  const auto setEdgeDefaults = [](EdgeSets &set) {
    set.safe.points[0] = 0.0f;
    set.safe.points[1] = 0.0f;
    set.safe.points[2] = 420.0f;
    set.safe.points[3] = 620.0f;

    set.danger.points[0] = 600.0f;
    set.danger.points[1] = 750.0f;
    set.danger.points[2] = 1000.0f;
    set.danger.points[3] = 1000.0f;
  };

  // Motor commands in PWM units (-255 to 255)
  const auto setMotorDefaults = [](MotorSets &set) {
    set.fastReverse.points[0] = -255.0f;
    set.fastReverse.points[1] = -255.0f;
    set.fastReverse.points[2] = -200.0f;
    set.fastReverse.points[3] = -140.0f;

    set.slowReverse.points[0] = -220.0f;
    set.slowReverse.points[1] = -140.0f;
    set.slowReverse.points[2] = -40.0f;

    set.stop.points[0] = -30.0f;
    set.stop.points[1] = 0.0f;
    set.stop.points[2] = 30.0f;

    set.slowForward.points[0] = 40.0f;
    set.slowForward.points[1] = 140.0f;
    set.slowForward.points[2] = 220.0f;

    set.fastForward.points[0] = 140.0f;
    set.fastForward.points[1] = 200.0f;
    set.fastForward.points[2] = 255.0f;
    set.fastForward.points[3] = 255.0f;
  };

  setDistanceDefaults(cfg.leftDistance);
  setDistanceDefaults(cfg.frontDistance);
  setDistanceDefaults(cfg.rightDistance);

  setEdgeDefaults(cfg.leftEdge);
  setEdgeDefaults(cfg.centerEdge);
  setEdgeDefaults(cfg.rightEdge);

  setMotorDefaults(cfg.leftMotor);
  setMotorDefaults(cfg.rightMotor);

  cfg.defuzzMethod = qlibs::fis::centroid;

  return cfg;
}

FuzzySumoController::FuzzySumoController() = default;

void FuzzySumoController::setConfig(const Config &config) {
  config_ = config;
  initialized_ = false;
}

void FuzzySumoController::begin(const Config &config) {
  config_ = config;
  initialize();
}

FuzzySumoController::Output FuzzySumoController::evaluate(const Input &input) {
  if (!initialized_) {
    initialize();
  }

  const real_t leftDist =
      clampValue(input.leftDistanceCm, config_.distanceRange[0], config_.distanceRange[1]);
  const real_t frontDist =
      clampValue(input.frontDistanceCm, config_.distanceRange[0], config_.distanceRange[1]);
  const real_t rightDist =
      clampValue(input.rightDistanceCm, config_.distanceRange[0], config_.distanceRange[1]);

  const real_t leftEdge =
      clampValue(input.leftEdgeNorm, config_.edgeRange[0], config_.edgeRange[1]);
  const real_t centerEdge =
      clampValue(input.centerEdgeNorm, config_.edgeRange[0], config_.edgeRange[1]);
  const real_t rightEdge =
      clampValue(input.rightEdgeNorm, config_.edgeRange[0], config_.edgeRange[1]);

  controller_.setInput(in_left_distance, leftDist);
  controller_.setInput(in_front_distance, frontDist);
  controller_.setInput(in_right_distance, rightDist);
  controller_.setInput(in_left_edge, leftEdge);
  controller_.setInput(in_center_edge, centerEdge);
  controller_.setInput(in_right_edge, rightEdge);

  Output output{};

  if (!controller_.fuzzify()) {
    return output;
  }

  if (!controller_.inference()) {
    return output;
  }

  if (!controller_.deFuzzify()) {
    return output;
  }

  output.leftMotorSpeed = controller_[out_left_motor];
  output.rightMotorSpeed = controller_[out_right_motor];

  return output;
}

void FuzzySumoController::initialize() {
  memset(ruleStrength_, 0, sizeof(ruleStrength_));

  controller_.setup(qlibs::fis::Mamdani, inputs_, kInputCount, outputs_, kOutputCount, inputMF_,
                    kInputMfCount, outputMF_, kOutputMfCount, kRules, kRuleCount, ruleStrength_);

  controller_.setupInput(in_left_distance, config_.distanceRange[0], config_.distanceRange[1]);
  controller_.setupInput(in_front_distance, config_.distanceRange[0], config_.distanceRange[1]);
  controller_.setupInput(in_right_distance, config_.distanceRange[0], config_.distanceRange[1]);

  controller_.setupInput(in_left_edge, config_.edgeRange[0], config_.edgeRange[1]);
  controller_.setupInput(in_center_edge, config_.edgeRange[0], config_.edgeRange[1]);
  controller_.setupInput(in_right_edge, config_.edgeRange[0], config_.edgeRange[1]);

  controller_.setupOutput(out_left_motor, config_.motorRange[0], config_.motorRange[1]);
  controller_.setupOutput(out_right_motor, config_.motorRange[0], config_.motorRange[1]);

  controller_.setupInputMF(in_left_distance, left_distance_close, qlibs::fis::trimf,
                           config_.leftDistance.close.data());
  controller_.setupInputMF(in_left_distance, left_distance_medium, qlibs::fis::trimf,
                           config_.leftDistance.medium.data());
  controller_.setupInputMF(in_left_distance, left_distance_far, qlibs::fis::trapmf,
                           config_.leftDistance.far.data());

  controller_.setupInputMF(in_front_distance, front_distance_close, qlibs::fis::trimf,
                           config_.frontDistance.close.data());
  controller_.setupInputMF(in_front_distance, front_distance_medium, qlibs::fis::trimf,
                           config_.frontDistance.medium.data());
  controller_.setupInputMF(in_front_distance, front_distance_far, qlibs::fis::trapmf,
                           config_.frontDistance.far.data());

  controller_.setupInputMF(in_right_distance, right_distance_close, qlibs::fis::trimf,
                           config_.rightDistance.close.data());
  controller_.setupInputMF(in_right_distance, right_distance_medium, qlibs::fis::trimf,
                           config_.rightDistance.medium.data());
  controller_.setupInputMF(in_right_distance, right_distance_far, qlibs::fis::trapmf,
                           config_.rightDistance.far.data());

  controller_.setupInputMF(in_left_edge, left_edge_safe, qlibs::fis::trapmf,
                           config_.leftEdge.safe.data());
  controller_.setupInputMF(in_left_edge, left_edge_danger, qlibs::fis::trapmf,
                           config_.leftEdge.danger.data());

  controller_.setupInputMF(in_center_edge, center_edge_safe, qlibs::fis::trapmf,
                           config_.centerEdge.safe.data());
  controller_.setupInputMF(in_center_edge, center_edge_danger, qlibs::fis::trapmf,
                           config_.centerEdge.danger.data());

  controller_.setupInputMF(in_right_edge, right_edge_safe, qlibs::fis::trapmf,
                           config_.rightEdge.safe.data());
  controller_.setupInputMF(in_right_edge, right_edge_danger, qlibs::fis::trapmf,
                           config_.rightEdge.danger.data());

  controller_.setupOutputMF(out_left_motor, left_motor_fast_reverse, qlibs::fis::trapmf,
                            config_.leftMotor.fastReverse.data());
  controller_.setupOutputMF(out_left_motor, left_motor_slow_reverse, qlibs::fis::trimf,
                            config_.leftMotor.slowReverse.data());
  controller_.setupOutputMF(out_left_motor, left_motor_stop, qlibs::fis::trimf,
                            config_.leftMotor.stop.data());
  controller_.setupOutputMF(out_left_motor, left_motor_slow_forward, qlibs::fis::trimf,
                            config_.leftMotor.slowForward.data());
  controller_.setupOutputMF(out_left_motor, left_motor_fast_forward, qlibs::fis::trapmf,
                            config_.leftMotor.fastForward.data());

  controller_.setupOutputMF(out_right_motor, right_motor_fast_reverse, qlibs::fis::trapmf,
                            config_.rightMotor.fastReverse.data());
  controller_.setupOutputMF(out_right_motor, right_motor_slow_reverse, qlibs::fis::trimf,
                            config_.rightMotor.slowReverse.data());
  controller_.setupOutputMF(out_right_motor, right_motor_stop, qlibs::fis::trimf,
                            config_.rightMotor.stop.data());
  controller_.setupOutputMF(out_right_motor, right_motor_slow_forward, qlibs::fis::trimf,
                            config_.rightMotor.slowForward.data());
  controller_.setupOutputMF(out_right_motor, right_motor_fast_forward, qlibs::fis::trapmf,
                            config_.rightMotor.fastForward.data());

  controller_.setDeFuzzMethod(config_.defuzzMethod);

  initialized_ = true;
}

