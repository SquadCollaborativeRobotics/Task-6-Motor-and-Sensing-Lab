Task 6 Motor/Sensor Lab
=================================

Team E - Squad Collaborative Robotics

![GUI](http://i.imgur.com/DkblNDl.png)


Installation
---
Move folders in libraries into arduino libraries folder.

* Windows: ``` Documents\Arduino\libraries\<PLACE_LIBRARIES_HERE>```


States
---

* 0 - Serial control mode, GUI can send motor position commands
* 1 - Ultrasonic sensor distance sets servo motor position
* 2 - LED Light sensor sets ??? of Stepper motor
* 3 - Potentiometer sets velocity of DC motor

GUI
---
The GUI was built using [Processing](http://www.processing.org/) and [ControlP5](http://www.sojamo.de/libraries/controlP5/).