/*
  StepperMotor.h - Library for commanding Stepper Motor
  Created by Alex Sher, Oct 8, 2013.
  Released into public domain.
*/

#ifndef StepperMotor_h
#define StepperMotor_h

class StepperMotor
{
public:
  StepperMotor(int dir_pin, int step_pin);
  void stepMotorDeg(int degrees, int speed);
private:
  int _dir_pin;
  int _step_pin;
  int _steps_per_rev;;
};

#endif
