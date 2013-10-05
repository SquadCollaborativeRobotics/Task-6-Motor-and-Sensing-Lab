/*
Task 2: Motor and Sensor Lab
Team E

Aaron Nye
Shawn Hanna
Alex Sher
Sameer Ansari

*/

//////////////////////////////////////////////////////////////////////////////
/// 
/// DEFINES
///
/// constants should be defined here, such as Pin numbers or math constants
///
/// ex: #define PI 3.14159
///
///	#define POT_PIN 9
///
//////////////////////////////////////////////////////////////////////////////

#define POT_PIN 9

//////////////////////////////////////////////////////////////////////////////
/// 
/// Global Variables
///
/// Create global variables here.  Try not to use too many, it's not a great
/// coding practice
///
/// All global variables should be prefaced with "g_"   - ex: g_motorState
///
//////////////////////////////////////////////////////////////////////////////


/*
**
** g_motorState specifies which motor mode we are running in, which picks
** which motor/sensor combination to operate (changes via button press)
**
*/

int g_motorState = 0;


void setup()
{
	pinMode(POT_PIN, INPUT);
}

void loop()
{
	double value = readPotentiometer();
}

double readPotentiometer()
{
	return analogRead(POT_PIN) / (double) 1023.0;
}