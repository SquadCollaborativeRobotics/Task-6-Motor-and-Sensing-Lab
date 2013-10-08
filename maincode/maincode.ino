/*
Task 6: Motor and Sensor Lab
Team E - Squad Collaborative Robotics

Aaron Nye
Shawn Hanna
Alex Sher
Sameer Ansari

*/

#include <Servo.h>
#include <ultrasonic.h>
#include <LEDLightSensor.h>
#include <Encoder.h>
#include <Timer.h>
#include <PID_v1.h>

//////////////////////////////////////////////////////////////////////////////
/// 
/// DEFINES
///
/// constants should be defined here, such as Pin numbers or math constants
///
/// ex: #define PI 3.14159
///
//////////////////////////////////////////////////////////////////////////////
// Ultrasonic config
#define ULTRASONIC_TRIG 8
#define ULTRASONIC_ECHO 4
#define ULTRASONIC_MIN_DIST_MM 0
#define ULTRASONIC_MAX_DIST_MM 300
// Servo config
#define SERVO_PIN 5
#define SERVO_MIN_POS 10
#define SERVO_MAX_POS 170

#define POT_PIN A0

// system state max 0-STATE_MAX, used for button state incrementing
#define STATE_MAX 3
#define BUTTON_PRESS_BOUNCE_DELAY_MS 25
#define BUTTON_PIN 2

// LED Light Sensor
#define LED_SENSOR_N_PIN 7
#define LED_SENSOR_P_PIN 6

// Encoder stuff
#define ENCODER_A_PIN 3
#define ENCODER_B_PIN 12
#define ENCODER_TIME_DELAY_MS 100
#define TICKS_PER_REVOLUTION 750

// DC Motor
#define DC_ENABLE_PIN 11
#define DC_DRIVE2_PIN 10
#define DC_DRIVE1_PIN 13

//////////////////////////////////////////////////////////////////////////////
/// 
/// Global Variables
///
/// Create global variables here.  Try not to use too many, it's not a great
/// coding practice
/// Note: Keep name order from least-most.
///
/// All global variables should be prefaced with "g_"   - ex: g_motorState
///
//////////////////////////////////////////////////////////////////////////////

// Ultrasonic sensor that returns readings in millimeters.
Ultrasonic g_UsonicSensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);

// Encoder
Encoder g_DCMotorEncoder(ENCODER_A_PIN, ENCODER_B_PIN);
Timer g_TimerEncoder;
float g_EncoderVelocity = 0; // Radians per second
long g_LastEncoderValue = 0;
double g_EncoderAngle = 0; // In revolutions (360.0 degrees = 1.0)

void updateEncoderReading() {
  // Get raw ticks
  long encoderValue = g_DCMotorEncoder.read();

  // Get position in revolutions (includes multiple revolutions)
//  Serial.println(encoderValue);
  g_EncoderAngle = (double)encoderValue / (double)TICKS_PER_REVOLUTION;
//  Serial.println(g_EncoderAngle);
  
  // Get velocity in ticks/sec
  g_EncoderVelocity = 1000.0*(float)(encoderValue-g_LastEncoderValue)/(float)ENCODER_TIME_DELAY_MS;

  // Convert to radians/sec
  g_EncoderVelocity /=  (double)TICKS_PER_REVOLUTION;
  
  // Set last known encoder ticks
  g_LastEncoderValue = encoderValue;
}

// DC Motor
double g_DCMotorVelocity = 0; // Set velocity in range -1000, 1000
double g_DCMotorGoalPosition = 0; // in revolutions (1.0 = 360 degrees)

// Use 20V for the DC Motor at these gaine values
#define MIN_DC_SPD 100
// (Input, Output, Setpoint, P, I, D, DIRECT)
PID g_DCMotorPID(&g_EncoderAngle,
                 &g_DCMotorVelocity,
                 &g_DCMotorGoalPosition,
                 2000.0,
                 5.0,
                 50,
                 DIRECT);

// LED light sensor setup
// Set it up like http://playground.arduino.cc/Learning/LEDSensor
//           + N PIN
//           |
//           <
//           > 100 ohm resistor
//           <
//           |
//           |
//         -----
//          / \  LED, maybe a 5mm, clear plastic is good
//         -----
//           |
//           |
//           + P PIN

LEDLightSensor g_LEDLightSensor(LED_SENSOR_N_PIN, LED_SENSOR_P_PIN);

// Servo controller
Servo g_Servo;

/*
**
** g_motorState specifies which motor mode we are running in, which picks
** which motor/sensor combination to operate (changes via button press)
**
** 0 - Serial Control
** 1 - Servo motor & Ultrasonic sensor
*/

int g_motorState = 0;

// software de-bounce last time for the button
long last_time_high = 0; 
boolean button_debounce_ignore = false;
int last_state = LOW;
int current_state = LOW;

// De-bounced button press interrupt for state switching.
void button_pressed() {
  if (abs(millis() - last_time_high) > BUTTON_PRESS_BOUNCE_DELAY_MS) {
    stopDC();
    Serial.print("State change from ");
    Serial.print(g_motorState, DEC);
    g_motorState++;
    if (g_motorState > STATE_MAX) {g_motorState = 0;}
    Serial.print(" -> ");
    Serial.println(g_motorState, DEC);
    last_time_high = millis();
  }
}

// Returns pot value from as double from 0 to 1.0
double readPotentiometer()
{
  return (double)analogRead(POT_PIN) / 1023.0;
}

// Set Velocity -1000 to 1000
void setDCVelocity(int vel) {
  vel = constrain(vel, -1000, 1000);
  if (vel < 0) {
    setDCMotor(abs(vel), -1);
  }
  else {
    setDCMotor(vel, 1);
  }
}

// Set speed 0 to 1000, dir -1 or 1
void setDCMotor(unsigned int spd, int dir)
{
  // Speed must positive, and in range 0 - 1000
  // Direction sets direction.
  spd = map(spd, 0, 1000, 0, 255);

  // dir = direction, 1 = forward, -1 = backward
  analogWrite(DC_ENABLE_PIN, spd);
  if (dir == 1 || dir == -1) {
    digitalWrite(DC_DRIVE1_PIN, dir > 0 ? LOW : HIGH);
    digitalWrite(DC_DRIVE2_PIN, dir > 0 ? HIGH : LOW);
  } 
  else {
    Serial.println("Weird direction given");
  }
}

// Stops DC motor
void stopDC() {
  analogWrite(DC_ENABLE_PIN, 0);
  digitalWrite(DC_DRIVE1_PIN, LOW);
  digitalWrite(DC_DRIVE2_PIN, LOW);
}

// PID move to degree value
void setDCtoDeg(int deg) {
  // Set goal point in revolutions
//  double rev = map((double)deg, 0.0, 360.0, 0.0, 1.0);
//  g_DCMotorGoalPosition = rev;
  g_DCMotorGoalPosition = (double)deg / 360.0;
  Serial.print("Current: ");
  Serial.println(g_EncoderAngle);
  Serial.print("Goal: ");
  Serial.println(g_DCMotorGoalPosition);
  
  // While not within 1 degrees of goal
  while ( abs(g_DCMotorGoalPosition - g_EncoderAngle) > 1.0/360.0) {
    Serial.print("Current: ");
    Serial.print(g_EncoderAngle);
    Serial.print(", Diff to goal: ");
    Serial.print(g_DCMotorGoalPosition - g_EncoderAngle);
    
    // Update encoder input val
    updateEncoderReading();
    
    // Update PID to set output velocity
    g_DCMotorPID.Compute();
    
    // Set velocity
    if (g_DCMotorVelocity >= 0 && g_DCMotorVelocity < MIN_DC_SPD) { g_DCMotorVelocity = MIN_DC_SPD; }
    else if (g_DCMotorVelocity < 0 && g_DCMotorVelocity > -MIN_DC_SPD) { g_DCMotorVelocity = -MIN_DC_SPD; }
    setDCVelocity(g_DCMotorVelocity);
    
    Serial.print(", Vel set to: ");
    Serial.println(g_DCMotorVelocity);
    
    delay(10);
  }
  stopDC();
}

////////////////////////////////////////////////////////////////////////////////
// DEMOS

// Run servo motor position based on ultrasonic sensor distance
void demoUltrasonicAndServo() {
  // Ultrasonic sensor + motor
  unsigned long usonic_dist_mm = g_UsonicSensor.getFilteredReading();

  g_Servo.write(map(usonic_dist_mm, 
                    ULTRASONIC_MIN_DIST_MM,
                    ULTRASONIC_MAX_DIST_MM,
                    SERVO_MIN_POS,
                    SERVO_MAX_POS));

  delay(50);
}

// Run stepper motor based on LED sensor reading
void demoLightAndStepper() {
  // Get Light brightness level (0-100 bright to dark)
  unsigned int led_val = g_LEDLightSensor.getFilteredReading();
  Serial.print("Light: ");
  Serial.println(led_val);
  
  // Set Motor position based on brightness level  
  g_Servo.write(map(led_val,0,100,SERVO_MIN_POS,SERVO_MAX_POS));
  
  delay(100);
}

void demoPotAndDC() {
  double potVal = readPotentiometer();
  // Set potVal to -1000,1000
  potVal = potVal*2000 - 1000;
  setDCVelocity(potVal);
  delay(20);
}

////////////////////////////////////////////////////////////////////////////////
// Serial events (only in state 0)

void serialEvent() {
  int pos, vel;
  // Only run in state 0
  if (g_motorState != 0) {return;}
  
  while (Serial.available()) {
    char c = (char)Serial.read();

    switch (c) {
      case 'a':
      // Servo position given 0-180
      pos = Serial.parseInt();
      pos = constrain(pos, 0, 180); // Not limited to min/max here.
      Serial.print("Setting servo to position ");
      Serial.println(pos);
      g_Servo.write(pos);
      break;
      
      case 'b':
      // Aaron serial command Stepper motor
      pos = Serial.parseInt();
      Serial.print("Setting stepper to ??? ");
      Serial.println(pos);
      break;
      
      case 'c':
      // Set velocity of DC motor
      vel = Serial.parseInt();
      Serial.print("Setting DC motor velocity to ");
      Serial.println(vel);
      setDCVelocity(vel);
      break;
      
      case 'd':
      // Use PID for position of DC motor shaft
      pos = Serial.parseInt();
      Serial.print("Setting DC motor position (degrees) to ");
      Serial.println(pos);
      setDCtoDeg(pos);
      break;
    }
  }
}

void printSensorInfo() {
  int ultrasonic = (int)g_UsonicSensor.getFilteredReading();
  int brightness = (int)g_LEDLightSensor.getFilteredReading();
  Serial.print('|');
  Serial.print(ultrasonic, DEC);
  Serial.print(' ');
  Serial.print(brightness);
  Serial.print(' ');
  Serial.print(g_DCMotorEncoder.read());
  Serial.print(' ');
  Serial.print(g_EncoderAngle * 360.0);
  Serial.print(' ');
  Serial.print(g_EncoderVelocity);
  Serial.print(' ');
  Serial.println(readPotentiometer());
}

void setup() {
  // Initialize serial communication.
  Serial.begin(9600);

  //// Pins
  Serial.println("Initializing pins...");
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  g_Servo.attach(SERVO_PIN);
  
  // Attach pin 2 (#0) interrupt for button press
  attachInterrupt(0, button_pressed, RISING);
  
  //// Sensors
  Serial.println("Initializing sensors...");
  
  // Initialize Ultrasonic sensor with min/max distance
  g_UsonicSensor.init(ULTRASONIC_MIN_DIST_MM, ULTRASONIC_MAX_DIST_MM);
  
  // Initialize LED Light Sensor
  g_LEDLightSensor.init();

  // Initialize encoder counter
  g_TimerEncoder.every(ENCODER_TIME_DELAY_MS, updateEncoderReading);
  
  // Initialize PID for DC Motor
  g_DCMotorPID.SetMode(AUTOMATIC);
  g_DCMotorPID.SetOutputLimits(-1000,1000);
  
  Serial.println("Initialized.");

  // TEMPORARY : TESTING SERVO/USONIC
  g_motorState = 0;
  
}

void loop() {
  switch (g_motorState) {
    case 0:
    // Handle serial commands
    break;
    
    case 1:
    // Servo motor and ultrasonic sensor demo
    demoUltrasonicAndServo();
    break;
    
    case 2:
    // Aaron function call
    demoLightAndStepper();
    break;
    
    case 3:
    // Shawn function call
    demoPotAndDC();
    break;
    
    default:
    break;
  }
  g_TimerEncoder.update();
  printSensorInfo();
  delay(100);
}
