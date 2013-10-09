/**
 * Team E : Task 6 Motor Sensor Control Lab GUI
 */
import processing.serial.*;
import controlP5.*;
import java.awt.Frame;

Serial port;

ControlP5 cp5;
final int black = color(0,0,0);
int ultrasonic=0;
int brightness=0;
int encoder=0;
float pot=0;
int deg;
int stepdeg=0;
int prevPos=0;
int prevVel=0;
int prevDeg=0;
int prevStepdeg=0;
int currState = 0;

// Second Window Handlers
PFrame f;
secondApplet second;
// Graph Position Marker
int xPos = 1;
// Graph value holder
float brightVal;

void setup() {
  size(500,350);
  noStroke();
  cp5 = new ControlP5(this);
  
  // Second Window
  f = new PFrame();
  
  // change the default font to Verdana
  PFont p = createFont("Verdana",11); 
  cp5.setControlFont(p);
  
  // add a horizontal sliders, the value of this slider will be linked
  // to variable 'sliderValue'
  
  // Current State
  cp5.addNumberbox("currState")
     .setPosition(10,10)
     .setSize(30,15)
     .setRange(0,4)
     .setValue(0)
     .setCaptionLabel("STATE")
     ;
  
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
  cp5.addSlider("encoder_angle")
     .setPosition(100,150)
     .setRange(0,360)
     .setValue(0)
     .setCaptionLabel("Encoder Angle (degrees)")
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
  cp5.addSlider("stepper")
     .setPosition(100,230)
     .setRange(-360,360)
     .setNumberOfTickMarks(11)
     .setValue(0)
     .setCaptionLabel("Stepper motor offset (degrees)");
     
  cp5.addSlider("dc")
     .setPosition(100,260)
     .setRange(-1000,1000)
     .setNumberOfTickMarks(11)
     .setValue(0)
     .setCaptionLabel("DC motor velocity");
  cp5.addSlider("dc_pos")
     .setPosition(100,270)
     .setRange(0,360)
     .setNumberOfTickMarks(9)
     .setValue(0)
     .setCaptionLabel("DC motor position (degrees)")
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
    // |ultrasonic brightness encoderTicks encoderAngle encoderVelocity potentiometer
    String[] sensor_readings = splitTokens(s, " ");
    if (sensor_readings.length == 6) {
      // Update all gui values
      cp5.controller("ultrasonic").setValue(int(sensor_readings[0]));
      cp5.controller("brightness").setValue(int(sensor_readings[1]));
      cp5.controller("encoder").setValue(Long.parseLong(sensor_readings[2]));
      cp5.controller("encoder_angle").setValue(float(sensor_readings[3]));
      cp5.controller("encoder_velocity").setValue(float(sensor_readings[4]));
      cp5.controller("pot").setValue(float(sensor_readings[5]));
      
      // Map brightness sensor value for grapher
      brightVal = map(int(sensor_readings[1]), 0, 100, 0, second.height);
      
    } else {
      println("You done goofed.");
    }
  }
  else if (s!= null && s.charAt(0) == '!') {
    print("Current State: ");
    currState = int(trim(s.substring(1)));
    println(currState);
    cp5.controller("currState").setValue(currState);
  }
}


void draw() {
  background(black);
  fill(brightness);
  rect(50,70,40,20);
  
  // Add to make sure graph gets redrawn
  second.redraw();
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

void stepper(int stepdeg) {
  if (stepdeg != prevStepdeg) {
    println("Offsetting Stepper by "+stepdeg);
    port.write("b");
    port.write(str(stepdeg));
    port.write('\n');
    prevStepdeg = stepdeg;
  }
}

void dc_pos(int deg) {
  if (deg != prevDeg) {
    println("Setting DC to degree "+deg);
    port.write("d");
    port.write(str(deg));
    port.write('\n');
    prevDeg = deg;
  }
}

// Class for Second Window
public class PFrame extends Frame {
  public PFrame() {
      super("Embedded PApplet");
      setBounds(100, 100, 400, 350);
      // Add Applet
      second = new secondApplet();
      add(second);
      // important!! do not remove
      second.init();
      show();
  }
}

public class secondApplet extends PApplet {
  // Initialize Properties for graph window
  public void setup(){
    size(400, 350);
    background(black);
    stroke(255, 0, 0);
    strokeWeight(2);
    noLoop();
  }
  public void draw(){
    // Draw new line at xPos with brightVal Height
    // brightVal is value mapped from 0-100 -> 0-height
    line(xPos, height, xPos, height-brightVal);
    //print("x = "+xPos);
    //println(" y = "+brightVal);
  
    // If rollover, reset and start again
    if(xPos >= width){
      xPos = 0;
      second.background(black);
    }  
    // else increment x value
    else{ xPos++; 
    }
  }
}
    
  
  
