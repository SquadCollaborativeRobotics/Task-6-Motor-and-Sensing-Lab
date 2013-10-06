/*
Task 2: Motor and Sensor Lab
Team E

Aaron Nye
Shawn Hanna
Alex Sher
Sameer Ansari

*/

#include <Servo.h>
#include <ultrasonic.h>

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
#define ULTRASONIC_TRIG 3
#define ULTRASONIC_ECHO 4
#define ULTRASONIC_MIN_DIST_MM 0
#define ULTRASONIC_MAX_DIST_MM 300
// Servo config
#define SERVO_PIN 5
#define SERVO_MIN_POS 10
#define SERVO_MAX_POS 170

/// Shawn's stuff
#define POT_PIN 9
#define MOTOR_DIR_PIN1 8
#define MOTOR_DIR_PIN2 7
#define MOTOR_SPEED_PIN 6


// system state max 0-STATE_MAX, used for button state incrementing
#define STATE_MAX 3
#define BUTTON_PRESS_BOUNCE_DELAY_MS 20
#define BUTTON 2

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

// De-bounced button press interrupt for state switching.
void button_pressed() {
  if (abs(millis() - last_time_high) > BUTTON_PRESS_BOUNCE_DELAY_MS) {

    Serial.print("State change from ");
    Serial.print(g_motorState, DEC);
    g_motorState++;
    if (g_motorState > STATE_MAX) {g_motorState = 0;}
    Serial.print(" to ");
    Serial.println(g_motorState, DEC);

    last_time_high = millis();
  }
}


////////////////////////////////////////////////////////////////////////////////
// DEMOS

// Run servo motor position based on ultrasonic sensor distance
void demoUltrasonicAndServo() {
  // Ultrasonic sensor + motor
  unsigned long usonic_dist_mm = g_UsonicSensor.getFilteredReading();

  usonic_dist_mm = constrain(usonic_dist_mm,
                             ULTRASONIC_MIN_DIST_MM,
                             ULTRASONIC_MAX_DIST_MM);

  g_Servo.write(map(usonic_dist_mm, 
                    ULTRASONIC_MIN_DIST_MM,
                    ULTRASONIC_MAX_DIST_MM,
                    SERVO_MIN_POS,
                    SERVO_MAX_POS));

  delay(50);
}

////////////////////////////////////////////////////////////////////////////////
// Serial events (only in state 0)

void serialEvent() {
  // Only run in state 0
  if (g_motorState != 0) {return;}
  
  while (Serial.available()) {
    char c = (char)Serial.read();

    switch (c) {
      case 'a':
      // Servo position given 0-180
      int pos = Serial.parseInt();
      pos = constrain(pos, 0, 180); // Not limited to min/max here.
      Serial.print("Setting servo to position ");
      Serial.println(pos);
      g_Servo.write(pos);
      break;
      
      case 'b':
      // Aaron serial command motor
      int pos = Serial.parseInt();
      break;
      
      case 'c':
      // Shawn serial command motor
      int pos = Serial.parseInt();
      break;
    }
  }
}


void setup() {
  // Initialize serial communication.
  Serial.begin(9600);

  //// Pins
  Serial.println("Initializing pins...");
  pinMode(POT_PIN, INPUT);
  pinMode(MOTOR_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN1, OUTPUT);
  pinMode(MOTOR_DIR_PIN2, OUTPUT);
  g_Servo.attach(SERVO_PIN);
  
  // Attach interrupt for the button (on pin 2 = 0)
  attachInterrupt(0, button_pressed, RISING);

  
  //// Sensors
  Serial.println("Initializing sensors...");
  
  // Initialize Ultrasonic sensor
  g_UsonicSensor.init();


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
    break;
    
    case 3:
    double newVal = readPotentiometer();
    setSpeed(newVal);
    break;
    
    default:
    break;
  }
  // Flush serial
}

double readPotentiometer()
{
  return analogRead(POT_PIN) / (double) 1023.0;
}

void setSpeed(double newVal)
{
  int direction = 0;
  if (newVal - 1023.0/2.0 < 0)
  {
    direction = 1;
  }
  int speed = Math.abs(newVal - 1023.0/2.0);
  analogWrite(MOTOR_SPEED_PIN, speed);
  if (direction == 0)
  {
    digitalWrite(MOTOR_DIR_PIN1, 1);
    digitalWrite(MOTOR_DIR_PIN2, 0);
  }
  else
  {
    digitalWrite(MOTOR_DIR_PIN1, 0);
    digitalWrite(MOTOR_DIR_PIN2, 1);
  }

}
