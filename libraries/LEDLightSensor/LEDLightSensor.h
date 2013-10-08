/*
  LEDLightSensor.h - Library for getting light level readings from an LED.
  Created by Sameer Ansari, Oct 10, 2013.
  Released into public domain.
*/

#ifndef LEDLightSensor_H
#define LEDLightSensor_H

class LEDLightSensor
{
public:
  LEDLightSensor(int N_pin, int P_pin);
  unsigned int getReading();
  unsigned int getFilteredReading();
  void init();

private:
  int _N_pin, _P_pin;
  unsigned int _last;
  unsigned int getRawDistance();
};

#endif
