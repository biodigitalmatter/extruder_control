// Install AccelStepper: Tools > Manage libraries > Search for and install "AccelStepper"
#include <AccelStepper.h>

enum AnalogReferenceType { AREF_DEFAULT,
                           AREF_INTERNAL,
                           AREF_EXTERNAL };

// SETTINGS
const int HOTEND_TEMP_DEGREES_C = 210;
const int EXTRUDER_RPM = 120;  // rotations per minute of stepper

// PINS

// Comm pins
const int DI_ROBOT_STEPPER_BACKWARDS_PIN = 6;
const int DI_ROBOT_STEPPER_FORWARDS_PIN = 7;
const int DI_ROBOT_HEAT_UP_PIN = 8;
const int DO_ROBOT_TEMP_REACHED_PIN = A0;

const int DO_24V_TO_5V_BOARD_PWR_PIN = 5;
const int MOTOR_DRIVER_PWR_PIN = 10;

// Stepper motor pins
const int DO_NC_ENABLE_PIN = 3;  // ENA - Enable when 0, disable when 1
const int DO_DIR_PIN = 11;       // DIR - Direction
const int DO_STEP_PIN = 12;      // STP/PUL - Step, Pulse

// hotend pins
const int DO_HEATING_PIN = 2;
const int AI_THERMISTOR_PIN = A5;

// THERMISTOR
const long TEMP_CONTROL_INTERVAL_MILLIS = 2000;  // interval at which to check temperature

const float THERMISTOR_SETUP_FIXED_R1_OHM = 100000.0;  // 100k Ohm reference resistor
const AnalogReferenceType ANALOG_REFERENCE_TYPE = AREF_INTERNAL;

// Obtained Steinhart-Hart values from:
// https://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
// https://download.lulzbot.com/retail_parts/Completed_Parts/100k_Semitec_NTC_Thermistor_235mm_KT-CP0110/GT-2-glass-thermistors.pdf
// beta-value (at 25 degrees): 4267K
// resistance (Ohm) | degrees (C)
// 353700           |   0
//   5556           | 100
//   439.3          | 200
const float SH_A = 0.8097317731e-03, SH_B = 2.11635527e-04, SH_C = 0.7066084133e-07;

unsigned long g_previous_millis = 0;  // will store last time PRINT was updated

// Hemera motor
const float STEP_ANGLE = 1.8;
const float GEAR_MULTIPLIER = 3.32;
const int MICROSTEP_MULTIPLIER = 16;
const bool INVERT_STEPPER_DIR = true;

// Create a new instance of the AccelStepper class:
AccelStepper g_stepper = AccelStepper(AccelStepper::DRIVER, DO_STEP_PIN, DO_DIR_PIN);

// FUNCTIONS

// Get the reference voltage based on the current analog reference type
float getReferenceVoltage() {
  if (ANALOG_REFERENCE_TYPE == INTERNAL) {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)
    return 1.1;  // 1.1V for ATmega328P, ATmega168, and ATmega328PB (Arduino Uno, Nano, and Pro Mini 5V)
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    return 1.1;  // 1.1V for ATmega32U4, ATmega1280, and ATmega2560 (Arduino Leonardo, Mega 2560)
#else
    return 3.3;  // 3.3V for other 3.3V boards
#endif
  } else if (ANALOG_REFERENCE_TYPE == DEFAULT) {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    return 5.0;  // 5.0V for ATmega32U4, ATmega1280, and ATmega2560 (Arduino Leonardo, Mega 2560)
#else
    return 3.3;  // 3.3V for other 3.3V boards
#endif
  } else {
    // For external reference, set the voltage manually (e.g., 3.3V or 5V)
    return 3.3;
  }
}

float readThermistorTemperature() {
  // TODO: Replace with <Thermistor.h>?
  // Read the analog value (0-1023) from the thermistor pin
  int analogValue = analogRead(AI_THERMISTOR_PIN);

  float referenceVoltage = getReferenceVoltage();

  // Convert the analog value to voltage (using the internal reference voltage)
  float Vout = (analogValue * referenceVoltage) / 1023.0;

  // Calculate the thermistor resistance
  float thermistor_ohm = THERMISTOR_SETUP_FIXED_R1_OHM * (referenceVoltage / Vout - 1.0);

  // Convert resistance to temperature using the Steinhart-Hart equation coefficients (c1, c2, c3)
  float T = 1.0 / (SH_A + SH_B * log(thermistor_ohm) + SH_C * pow(log(thermistor_ohm), 3));

  // Convert temperature from Kelvin to Celsius
  float Tc = T - 273.15;

  return Tc;
}

float rpm2StepsPerSec(int rpm, float step_angle, int microstep_multiplier, float gear_multiplier) {
  float steps_per_rev = 360 / step_angle;

  float actual_steps_per_rev = steps_per_rev * microstep_multiplier * gear_multiplier;
  float steps_per_min = rpm * actual_steps_per_rev;

  return steps_per_min / 60.0;
}

const int STEPS_PER_SEC = round(rpm2StepsPerSec(EXTRUDER_RPM, STEP_ANGLE, MICROSTEP_MULTIPLIER, GEAR_MULTIPLIER));

void setup() {
  analogReference(ANALOG_REFERENCE_TYPE);

  // Set the maximum speed in steps per second:
  g_stepper.setMaxSpeed(3200);
  g_stepper.setSpeed(0);

  g_stepper.setPinsInverted(INVERT_STEPPER_DIR, false, false);

  // Add pinMode for enable pin and set it to LOW to enable the stepper driver
  pinMode(DO_NC_ENABLE_PIN, OUTPUT);
  digitalWrite(DO_NC_ENABLE_PIN, LOW);

  // Setup Thermistor
  Serial.begin(9600);

  // setup 24V to 5V board pwr
  pinMode(DO_24V_TO_5V_BOARD_PWR_PIN, OUTPUT);
  digitalWrite(DO_24V_TO_5V_BOARD_PWR_PIN, HIGH);

  // set up motor driver pwr
  pinMode(MOTOR_DRIVER_PWR_PIN, OUTPUT);
  digitalWrite(MOTOR_DRIVER_PWR_PIN, HIGH);

  // setup robot input pin
  pinMode(DI_ROBOT_STEPPER_BACKWARDS_PIN, INPUT_PULLUP);
  pinMode(DI_ROBOT_STEPPER_FORWARDS_PIN, INPUT_PULLUP);
  pinMode(DI_ROBOT_HEAT_UP_PIN, INPUT_PULLUP);
  // setup robot output pin
  pinMode(DO_ROBOT_TEMP_REACHED_PIN, OUTPUT);
  digitalWrite(DO_ROBOT_TEMP_REACHED_PIN, LOW);

  // hotend pins
  pinMode(DO_HEATING_PIN, OUTPUT);
  digitalWrite(DO_HEATING_PIN, LOW);

  pinMode(AI_THERMISTOR_PIN, INPUT);

  // led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  bool forwards = digitalRead(DI_ROBOT_STEPPER_FORWARDS_PIN) == HIGH;
  //bool backwards_signal = digitalRead(DI_ROBOT_STEPPER_BACKWARDS_PIN) == HIGH;
  bool backwards = false;

  int direction = 0;

  if (forwards && !backwards) {
    direction = 1;
  } else if (!forwards && backwards) {
    direction = -1;
  }

  g_stepper.setSpeed(STEPS_PER_SEC * direction);

  g_stepper.runSpeed();  // Rotate the g_stepper motor

  // make sure that heater is off when signal is low
  if (digitalRead(DI_ROBOT_HEAT_UP_PIN == LOW)) {
    digitalWrite(DO_HEATING_PIN, LOW);
  } else { // if signal is high
    unsigned long current_millis = millis();
    // and enough time has passed since last check
    if (current_millis - g_previous_millis > TEMP_CONTROL_INTERVAL_MILLIS) { 
      g_previous_millis = current_millis;

      // Read the thermistor temperature
      float temperature = readThermistorTemperature();

      // TODO: Add PID controller and send PWM to MOSFET
      int pinstate = temperature < HOTEND_TEMP_DEGREES_C ? HIGH : LOW;
      digitalWrite(DO_HEATING_PIN, pinstate);
    }
  }
}
