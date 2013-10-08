#include "StepperMotor.h"
#include "Arduino.h"

//Initialize Stepper Class
StepperMotor::StepperMotor(int dir_pin, int step_pin){
  _dir_pin = dir_pin;
  _step_pin = step_pin;
  _steps_per_rev = 400;
}

void StepperMotor::init(){
  pinMode(_dir_pin, OUTPUT);
  pinMode(_step_pin, OUTPUT);
}

// Function takes stepper motor rotation in degrees (+ or -) and a speed in
// rpm and turns the motor the desired number of degrees
void StepperMotor::stepMotorDeg(int degrees, int speed){

  // Set dir pin based on num degrees
  int dir = degrees < 0 ? LOW : HIGH;
  digitalWrite(_dir_pin, dir);

  // Calculate number of steps needed
  int steps = (double) degrees * (double) _steps_per_rev / 360.0

  // Calculate delay time based on speed
  int delay_time_min = speed * _steps_per_rev
  // divide by (60 * 1000) to get ms
  int delay_time_ms = (double) delay_time_min / 60000.0

  for(int i=0; i<steps; i++){
    digitalWrite(_step_pin, HIGH);
    delay(delay_time_ms / 2);
    digitalWrite(_step_pin, LOW);
    delay(delay_time_ms / 2);
  }
}  
    

