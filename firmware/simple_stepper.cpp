#include <Arduino.h>
#include <AccelStepper.h>

//defines pins
const int O_PIN_PULSE = 8; // PUL - Pulse
const int O_PIN_DIRECTION = 9;  // DIR - Direction

const int I_PIN_ENABLE = 7;

// NEMA 17HS1070-C5X 1.8 degrees step angle ==> 360/1.8 = 200, so 200 steps per rev
// Lowest micro step setting on DM542 is 400 steps per rev. So we need to send 400 steps for one rev before gear
// Motor is 5:1 so so 2000 steps is required to turn the auger a full revolution 
// Prime extruder fast from lutum printer runs at one rev per sec
const float STEPS_PER_SEC = 1000;

// "Speeds of more than 1000 steps per second are unreliable." --accelstepper docs
// But it still seems fine
const float MAX_STEPS_PER_SEC = 1000;

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, O_PIN_PULSE, O_PIN_DIRECTION);

void setup()
{
  stepper.setMaxSpeed(MAX_STEPS_PER_SEC);
  stepper.setSpeed(STEPS_PER_SEC);

  pinMode(I_PIN_ENABLE, INPUT_PULLUP);
}

void loop()
{
  if (digitalRead(I_PIN_ENABLE))
  {
    stepper.runSpeed();
  }
}
