#include "Ultrasonic.h"
#include "Arduino.h"

// Maximum effective range is 4meters = 4000mm
// 4000mm = uptime * 17 / 100
// 4000 * 100 / 17 = max uptime
// 23529 microseconds maximum time to wait.
// This translates to a little under 5000 iterations on arduino uno
// Choosing to iterate on this instead of calling micros() each time, 
// which could reduce precision of distance reading
#define ULTRASONIC_MAX_WAIT_COUNT 5000

// Initialize sensor
Ultrasonic::Ultrasonic(int trigger_pin, int echo_pin) {
  _trigger_pin = trigger_pin;
  _echo_pin = echo_pin;
}

void Ultrasonic::init() {
  // Set up pins
  pinMode(_trigger_pin, OUTPUT);
  pinMode(_echo_pin, INPUT);
}

// Wrapper on getting sensor data.
unsigned long Ultrasonic::getReading() {
  return getRawDistance();
}

// Uses an ultrasonic sensor to get the measured distance in millimeters.
unsigned long Ultrasonic::getRawDistance() {
  unsigned long low_start_time, high_start_time, uptime, c;
  
  // Flush sensor LOW for a second
  digitalWrite(_trigger_pin, LOW);
  delayMicroseconds(2);
  
  // Pulse for 10 us
  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);
  
  // Wait until echo signal is up.
  c = 0;
  low_start_time = micros();
  while (digitalRead(_echo_pin) == LOW && c++ < 1000) {}
  if (micros()-low_start_time < 400) {
    // Sainsmart Ultrasonic only returns valid data
    // after being low for over 400 microseconds.
    return 4000;
  }
  
  // Measure echo signal uptime in microseconds.
  c = 0;
  high_start_time = micros();
  while(digitalRead(_echo_pin) == HIGH && c++ < ULTRASONIC_MAX_WAIT_COUNT) {}
  uptime = micros() - high_start_time;
  
  // Distance in meters = (duration in seconds) * (speed of sound m/s) / 2
  // Distance in cm = (t * 1e-6) * (340 * 1e2) / 2 = t * 17/1000
  // Distance in millimeters = (t * 1e-6) * (340 * 1e3) / 2 = t * 17/100
  
  // Return distance in mm, sensor is supposedly accurate to 0.3cm = 3mm
  // Clamp to 4000mm, which is 4m or maximum effective range of this sensor.
  return min(4000, uptime * 17 / 100);
}
