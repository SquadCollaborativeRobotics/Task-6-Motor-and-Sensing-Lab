/**
 * Team E : Task 6 Motor Sensor Control Lab GUI
 */
import processing.serial.*;
import controlP5.*;

Serial port;

ControlP5 cp5;
final int black = color(0,0,0);
int ultrasonic=0;
int brightness=0;
int encoder=0;
float pot=0;
int prevPos=0;
int prevVel=0;

void setup() {
  size(500,350);
  noStroke();
  cp5 = new ControlP5(this);
  
  // change the default font to Verdana
  PFont p = createFont("Verdana",11); 
  cp5.setControlFont(p);
  
  // add a horizontal sliders, the value of this slider will be linked
  // to variable 'sliderValue'
  
  // Sensor sliders 
  cp5.addSlider("ultrasonic")
     .setPosition(100,50)
     .setRange(0,300)
     .setCaptionLabel("Ultrasonic Sensor (millimeters)")
     ;
  cp5.addSlider("brightness")
     .setPosition(100,80)
     .setRange(0,100)
     .setCaptionLabel("Brightness")
     ;
     
  cp5.addSlider("pot")
     .setPosition(100,110)
     .setRange(0,1)
     .setCaptionLabel("Potentiometer")
     ;
  
  cp5.addSlider("encoder")
     .setPosition(100,140)
     .setRange(-100000,100000)
     .setValue(0)
     .setCaptionLabel("Encoder")
     ;
  cp5.addSlider("encoder_velocity")
     .setPosition(100,160)
     .setRange(-10,10)
     .setValue(0)
     .setCaptionLabel("Encoder Velocity (RPS)")
     ;
     
  // Motor sliders
  cp5.addSlider("servo")
     .setPosition(100,200)
     .setRange(0,180)
     .setNumberOfTickMarks(11)
     .setValue(90)
     .setCaptionLabel("Servo motor position");
     ;
  cp5.addSlider("stepper")
     .setPosition(100,230)
     .setRange(-10,10)
     .setNumberOfTickMarks(11)
     .setValue(0)
     .setCaptionLabel("Stepper motor velocity");
     ;
  cp5.addSlider("dc")
     .setPosition(100,260)
     .setRange(-1000,1000)
     .setNumberOfTickMarks(11)
     .setValue(0)
     .setCaptionLabel("DC motor velocity");
     ;
  
  // Set up serial port
  println(Serial.list());
  if (Serial.list().length > 0) {
    port = new Serial(this, Serial.list()[0], 9600);
    port.bufferUntil('\n');
  } 
  else {
    port = null;
  }

}

void serialEvent(Serial port) {
  String s = port.readStringUntil('\n');
  if (s != null && s.charAt(0) == '|') {
    s = trim(s.substring(1));
    // |ultrasonic brightness encoder encoderVelocity potentiometer
    String[] sensor_readings = splitTokens(s, " ");
    if (sensor_readings.length == 5) {
      cp5.controller("ultrasonic").setValue(int(sensor_readings[0]));
      cp5.controller("brightness").setValue(int(sensor_readings[1]));
      cp5.controller("encoder").setValue(Long.parseLong(sensor_readings[2]));
      cp5.controller("encoder_velocity").setValue(float(sensor_readings[3]));
      cp5.controller("pot").setValue(float(sensor_readings[4]));
    } else {
      println("You done goofed.");
    }
  }
}


void draw() {
  background(black);

  fill(brightness);
  rect(50,70,40,20);
}

void servo(int pos) {
  
  if (pos != prevPos) {
    println("Setting Servo to "+pos);
    port.write("a");
    port.write(str(pos));
    port.write('\n');
    prevPos = pos;
  }
}

void dc(int vel) {
  if (vel != prevVel) {
    println("Setting DC to "+vel);
    port.write("c");
    port.write(str(vel));
    port.write('\n');
    prevVel = vel;
  }
}
