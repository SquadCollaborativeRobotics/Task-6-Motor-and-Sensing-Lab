/*
Task 2: Motor and Sensor Lab
Team E

Aaron Nye
Shawn Hanna
Alex Sher
Sameer Ansari

*/

#include <ultrasonic.h>

//////////////////////////////////////////////////////////////////////////////
/// 
/// DEFINES
///
/// constants should be defined here, such as Pin numbers or math constants
/// Note: Keep pin order from least-most.
///
/// ex: #define PI 3.14159
///
//////////////////////////////////////////////////////////////////////////////

#define ULTRASONIC_TRIG 2
#define ULTRASONIC_ECHO 3
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

/*
**
** g_motorState specifies which motor mode we are running in, which picks
** which motor/sensor combination to operate (changes via button press)
**
*/

int g_motorState = 0;


void setup() {
  // Initialize serial communication.
  Serial.begin(9600);

  //// Pins
  Serial.println("Initializing pins...");
  pinMode(POT_PIN, INPUT);

  
  //// Sensors
  Serial.println("Initializing sensors...");
  
  // Initialize Ultrasonic sensor
  g_UsonicSensor.init();


  Serial.println("Initialized.");
}

void loop() {

  //// Sam
  // Ultrasonic sensor + motor
  unsigned long usonic_dist_mm = g_UsonicSensor.getReading();
}
