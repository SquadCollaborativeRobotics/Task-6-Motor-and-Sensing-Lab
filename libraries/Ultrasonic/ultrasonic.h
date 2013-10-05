/*
  Ultrasonic.h - Library for getting ultrasonic sensor readings.
  Created by Sameer Ansari, Aug 26, 2013.
  Released into public domain.
*/

#ifndef Ultrasonic_h
#define Ultrasonic_h

class Ultrasonic
{
public:
  Ultrasonic(int trigger_pin, int echo_pin);
  unsigned long getReading();
  unsigned long getFilteredReading();
  void init();

private:
  int _trigger_pin, _echo_pin;
  unsigned long _last;
  unsigned long getRawDistance();
};

#endif
