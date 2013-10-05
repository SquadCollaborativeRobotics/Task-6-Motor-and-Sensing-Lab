#include "LEDLightSensor.h"
#include "Arduino.h"

#define LEDLightSensor_MAX_WAIT_COUNT 30000
#define LEDLightSensor_FILTER_ALPHA 3/10
#define LEDLightSensor_MIN_THRESHOLD 5000
#define LEDLightSensor_MAX_THRESHOLD 20000

// Initialize sensor
LEDLightSensor::LEDLightSensor(int N_pin, int P_pin) {
  _N_pin = N_pin;
  _P_pin = P_pin;
}

void LEDLightSensor::init() {
  // No setup needed.
}

// Wrapper on getting sensor data.
unsigned int LEDLightSensor::getReading() {
  unsigned int val = getRawDistance();
  
  // Constraing brightness levels
  val = constrain(val,
                  LEDLightSensor_MIN_THRESHOLD,
                  LEDLightSensor_MAX_THRESHOLD);
  
  // Re-map LED brightness to 0-100 scale, 0 being bright, 100 being dark
  val = map(val,
            LEDLightSensor_MIN_THRESHOLD,
            LEDLightSensor_MAX_THRESHOLD,
            0,
            100);
  return val;
}

unsigned int LEDLightSensor::getFilteredReading() {
  int diff = (int)getReading() - (int)_last;
  _last += (unsigned int)(diff * LEDLightSensor_FILTER_ALPHA);
  return _last;
}

// Uses an LED to measure time to discharge LED capacitance
// which is a function of light level
unsigned int LEDLightSensor::getRawDistance() {
  unsigned int j;

  // Apply reverse voltage, charge up the pin and led capacitance
  pinMode(_N_pin, OUTPUT);
  pinMode(_P_pin, OUTPUT);
  digitalWrite(_N_pin, HIGH);
  digitalWrite(_P_pin, LOW);

  // Isolate the N pin end of the diode
  pinMode(_N_pin, INPUT);
  digitalWrite(_N_pin, LOW);  // turn off internal pull-up resistor

  // Count how long it takes the diode to bleed back down to a logic zero
  for (j = 0; j < LEDLightSensor_MAX_WAIT_COUNT; j++) {
    if (digitalRead(_N_pin)==0) break;
  }
  return j;
}
