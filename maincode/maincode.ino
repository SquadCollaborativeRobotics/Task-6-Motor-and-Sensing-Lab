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
float g_EncoderVelocity = 0;
long g_LastEncoderValue = 0;

void updateEncoderReading() {
  long encoderValue = g_DCMotorEncoder.read();
  g_EncoderVelocity = 1000.0*(float)(encoderValue-g_LastEncoderValue)/(float)ENCODER_TIME_DELAY_MS;
  g_LastEncoderValue = encoderValue;
}

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

double readPotentiometer()
{
  return analogRead(POT_PIN) / (double) 1023.0;
}

void setDCVelocity(unsigned int spd, int dir)
{
  // Speed must positive, and in range 0 - 1000
  // Direction sets direction.
  spd = map(spd, 0, 1000, 0, 255);

  // dir = direction, 1 = forward, -1 = backward
  analogWrite(DC_ENABLE_PIN, spd);
  if (dir == 1 || dir == -1) {
    digitalWrite(DC_DRIVE1_PIN, dir > 0 ? HIGH : LOW);
    digitalWrite(DC_DRIVE2_PIN, dir > 0 ? LOW : HIGH);
  } 
  else {
    Serial.println("Weird direction given");
  }
}

void stopDC() {
  digitalWrite(DC_DRIVE1_PIN, HIGH);
  digitalWrite(DC_DRIVE2_PIN, LOW);
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
  setDCVelocity(500.0, 1);
  delay(10);
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
      // Aaron serial command motor
      pos = Serial.parseInt();
      break;
      
      case 'c':
      // Shawn serial command motor
      vel = Serial.parseInt();
      if (vel < 0) {
        setDCVelocity(abs(vel), -1);
      } else {
        setDCVelocity(vel, 1);
      }
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
  Serial.println(g_EncoderVelocity / (float)TICKS_PER_REVOLUTION);
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
