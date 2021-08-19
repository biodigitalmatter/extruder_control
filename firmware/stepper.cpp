// Stepper library created by Mike McCauley, install from Tools > Manage Libraries
#include <Arduino.h>
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <extruder_control/Settings.h>

//defines pins
const int O_PIN_PULSE = 7;
const int O_PIN_DIRECTION = 4;

const int I_PIN_ENABLE = 8;

// NEMA 17HS1070-C5X 1.8 degrees step angle ==> 360/1.8 = 200, so 200 steps per rev
// Lowest micro step setting on DM542 is 400 steps per rev. So we need to send 400 steps for one rev
// This is before gear ratio
int steps_per_sec = 400;

// "Speeds of more than 1000 steps per second are unreliable." --accelstepper docs
const float MAX_STEPS_PER_SEC = 1000;

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, O_PIN_PULSE, O_PIN_DIRECTION);


void messageCb(const std_msgs::UInt16& speed_msg){
  stepper.setSpeed(speed_msg.data);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> sub("extruder_speed", &messageCb );

void setup()
{
  pinMode(I_PIN_ENABLE, INPUT_PULLUP);
  stepper.setMaxSpeed(MAX_STEPS_PER_SEC);
  stepper.setSpeed(steps_per_sec);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  if (digitalRead(I_PIN_ENABLE))
  {
    stepper.runSpeed();  
  }
  nh.spinOnce();
}
