# Fuzzy Controller Overview

This project combines a modular mini-sumo architecture with a Mamdani inference engine built on top of `qlibs::fis::instance`. Rules are described declaratively while fuzzification, aggregation, and centroid defuzzification are delegated to the qLibs++ library.

## Architecture

- `sumo.ino`: defines hardware constants, runs the state machine, calibrates sensors, gathers readings, and delegates fuzzy evaluation plus motor commands.
- `FuzzySumoController.h/.cpp`: wraps `qlibs::fis::instance`, configures inputs, outputs, and membership functions through `Config::makeDefault()`, and exposes `evaluate()` to obtain crisp PWM values.
- `MotorController`: lightweight class around the TB6612FNG driver that applies `move(int leftSpeed, int rightSpeed)`.
- Reference documentation: [qlibs::fis](https://kmilo17pet.github.io/qlibs-cpp/qfis_desc.html#qfis_buildfis) for nomenclature and API details.

## State Machine (button on D9)

| State | User action | Description |
| --- | --- | --- |
| `CALIBRATING_WHITE` | First press | Averages IR readings on the white edge and stores `irMinValues`. |
| `CALIBRATING_BLACK` | Second press | Averages IR readings on the dohyo surface and stores `irMaxValues`. |
| `READY_TO_START` | Third press | Launches the mandatory 5 s countdown before combat. |
| `IN_COMBAT` | Press during combat | Runs the fuzzy logic and drives motors. Another press moves to `STOPPED`. |
| `STOPPED` | Press | Halts the motors and returns to `READY_TO_START` without recalibration. |

Button debounce is handled with a 50 ms delay and edge-trigger events.

## IR Calibration and Normalisation

- Place the robot on white, press the button once, and keep it still: a window of `kCalibrationCaptureMs` (5 s by default) collects continuous samples.
- When finished, firmware switches to `CALIBRATING_BLACK`. Move to black, press again, and wait another 5 s to capture the maximum values.
- `calibrateIRSensors()` accumulates sums and sample counts during the window. `finalizeWhiteCalibration()` / `finalizeBlackCalibration()` compute averages and update `irMinValues` / `irMaxValues`; if the maximum falls below the minimum we force a small gap.
- `readNormalizedIR(pin)` returns a 0–1000 score (0 ≈ safe black, 1000 ≈ white edge).

## Ultrasonic Reading

`getUltrasonicDistance(trigger, echo)` emits a 10 µs pulse, uses `pulseIn` with a 25 ms timeout, clamps the result to 0–350 cm, and feeds it directly to the fuzzy engine.

## FuzzySumoController

### Default Membership Sets

**Distance (cm):**

| Set | Points |
| --- | --- |
| `Close` | 0, 15, 35 |
| `Medium` | 25, 55, 85 |
| `Far` | 70, 120, 220, 320 |

**Edge (normalised 0–1000):**

| Set | Points |
| --- | --- |
| `Safe` | 0, 0, 420, 620 |
| `Danger` | 600, 750, 1000, 1000 |

**Motor speed (PWM):**

| Set | Points | Shape |
| --- | --- | --- |
| `FastReverse` | -255, -255, -200, -140 | Trapezoid (left shoulder) |
| `SlowReverse` | -220, -140, -40 | Triangle |
| `Stop` | -30, 0, 30 | Triangle |
| `SlowForward` | 40, 140, 220 | Triangle |
| `FastForward` | 140, 200, 255, 255 | Trapezoid (right shoulder) |

Defuzzification relies on qLibs++’s `fis::centroid` method executed by the Mamdani engine.

### Rule Base

1. Right edge `Danger` ⟹ left `FastReverse`, right `SlowReverse`.
2. Left edge `Danger` ⟹ left `SlowReverse`, right `FastReverse`.
3. Centre edge `Danger` ⟹ both `FastReverse`.
4. Front distance `Close` ⟹ both `FastForward`.
5. Right distance `Close` ⟹ left `FastForward`, right `SlowForward`.
6. Left distance `Close` ⟹ left `SlowForward`, right `FastForward`.
7. All distances `Far` ⟹ left `SlowForward`, right `SlowReverse` (search spin).

Rule antecedents use the `min` operator; aggregation across rules targeting the same output set uses `max`.

## Motor Control

`MotorController` configures the TB6612FNG pins at setup time. The `move()` method clamps commands to ±255, sets direction via `IN1/IN2`, and writes PWM to the `pwm` pin. `stop()` disables both channels and pulls `STBY` low when defined.

## Tuning Guide

- **Distances**: adjust `Config::distance` points to match the real HC-SR04 response; widen the `Close` triangle if noise is significant.
- **Edge sensors**: tweak the `Danger` trapezoid to balance sensitivity versus false positives; increase the lower points if sensors read high even over black.
- **Outputs**: modify membership points to reshape acceleration; changing the slope of `SlowForward`/`SlowReverse` alters blending.
- **Search behaviour**: rule 7 can be made more aggressive by tweaking the `SlowForward` / `SlowReverse` sets or adding extra rules for the `Medium` distance band.

## References

- qLibs++ FIS API for terminology and usage: [official documentation](https://kmilo17pet.github.io/qlibs-cpp/qfis_desc.html#qfis_buildfis)
