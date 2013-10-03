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
#define ULTRASONIC_TRIG 2
#define ULTRASONIC_ECHO 4
#define ULTRASONIC_MIN_DIST_MM 0
#define ULTRASONIC_MAX_DIST_MM 300
// Servo config
#define SERVO_PIN 3
#define SERVO_MIN_POS 45
#define SERVO_MAX_POS 135

#define POT_PIN 9

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

void setup() {
  // Initialize serial communication.
  Serial.begin(9600);

  //// Pins
  Serial.println("Initializing pins...");
  pinMode(POT_PIN, INPUT);
  g_Servo.attach(SERVO_PIN);

  
  //// Sensors
  Serial.println("Initializing sensors...");
  
  // Initialize Ultrasonic sensor
  g_UsonicSensor.init();


  Serial.println("Initialized.");

  // TEMPORARY : TESTING SERVO/USONIC
  g_motorState = 1;
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
    
    default:
    break;
  }
  // Flush serial
}


