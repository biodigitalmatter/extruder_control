#include <Arduino.h>
#include <AccelStepper.h>

//define pins
const int O_PIN_PULSE = 9; // PUL - Pulse
const int O_PIN_DIRECTION = 7;  // DIR - Direction
const int I_PIN_RUN = 2;

const int EXTRUDER_RPM = 120;

// NEMA 17HS1070-C5X: 1.8 degrees step angle
const float STEP_ANGLE = 1.8;

// microsteps per rev => microstep multiplier:
// steps_per_rev_microstepping => steps_per_rev_microstepping/steps_per_rev = microstep multiplier
// e.g:
// 400 => 400/200 = 2
// or (steps_per_rev/microsteps_per_rev)^-1 = microstep_multiplier
const int MICROSTEP_MULTIPLIER = 1;

// NEMA 17HS1070-C5X: 5:1
const int GEAR_MULTIPLIER = 5;

int STEPS_PER_SEC;

// "Speeds of more than 1000 steps per second are unreliable." --accelstepper docs
// But it still seems fine
const float MAX_STEPS_PER_SEC = 1000;

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, O_PIN_PULSE, O_PIN_DIRECTION);

void setup()
{
  STEPS_PER_SEC = RPM_TO_STEPS_PER_SEC(EXTRUDER_RPM, STEP_ANGLE, MICROSTEP_MULTIPLIER, GEAR_MULTIPLIER);

  stepper.setMaxSpeed(MAX_STEPS_PER_SEC);
  stepper.setSpeed(STEPS_PER_SEC);

  pinMode(I_PIN_RUN, INPUT_PULLUP);
}

void loop()
{
  if (digitalRead(I_PIN_RUN))
  {
    stepper.runSpeed();
  }
}

int RPM_TO_STEPS_PER_SEC(int rpm, float step_angle, int microstep_multiplier, int gear_multiplier) {
  int steps_per_rev = int(360 / step_angle);

  int actual_steps_per_rev = steps_per_rev * microstep_multiplier * gear_multiplier;
  int steps_per_min = rpm * actual_steps_per_rev;

  return steps_per_min / 60;
}
