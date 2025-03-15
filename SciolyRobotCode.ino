#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
#include "MotionControl.h"

Romi32U4ButtonA buttonA; 
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

/*
General units: 
cm, cm/s, cm/s^2
degrees, degrees/s, degrees/s^2

Error Margins:
+/- <1 degree
+/- <0.5 cm

Notes
- Feedforward term, max acceleration for PIDs may need adjustment depending on suface, and static frcition.
- PID Constants are set at the top of MotionControl.cpp
- Check battery power. 
- Only change constants if absolutely necessary. 

Robot start usage:
1. Press power button and wait until orange LED turns on and then off for initialization.
Do not move the robot in any way while orange LED is on during setup. If robot is moved, restart. 
2. Press any of the function buttons to run their associated code (A, B, C). 

Button usages:
A - n/a
B - n/a
C - Testing

Robot orientation: the back of the robot is the side with the function buttons. The front of the robt has a bigger free area. 

Drive Function Usage (paste in loop section): 
void forward(float distance_CM);
void turnRight(float angle = 90);
void turnLeft(float angle = 90); 

TO DO:
- profiled pid,
- time limits?'


===============
Robot Tour Settings
===============
Max Speed: 130 units
Turbo: no
Driving PID: 0.4, 0, 0.1, 22, 1
Heading PID: 2, 0, 0.3, 23, 0.5
Correction PID: 4, 0, 0.1, 0, 0.5
Max Accel limiter: 5 units/15ms

===============
Electric Vehicle Settings
===============
Max Speed:
Turbo:
Driving PID:
Heading PID:
Correction PID:
Max Accel limiter:


*/
unsigned long startTime; // ms
float totalTime; // s

void setup()
{
  Serial.begin(9600);
  initMotionControl();

}

void loop(){

  if(buttonA.isPressed()){
    Serial.println("=========== Button A Pressed - New Command Sequence Started ===========");
    delay(1000);
    ledGreen(true);
    startTime = millis();

    
    // ================
    // INSERT MAIN DRIVING CODE BELOW
    //=================
 
    forward(25);
    turnLeft();
    forward(50);
    forward(-50);
    turnRight(); // reached A


//
    forward(50); 
    turnRight();
    forward(50);
    turnRight();
    forward(50);
    turnLeft();
    forward(50);
    forward(-50);
    turnLeft(); 
    
    forward(50);
    turnLeft();
    forward(100);

//

    turnRight();
    forward(160);
    
    turnRight();
    forward(100);
    turnRight();
    forward(50);
    turnLeft();
    forward(50);
    turnRight();
    forward(50);
    forward(-50); //d

    turnRight();
    forward(50);
    turnRight();
    forward(50);
    turnRight(); // one before end

    forward(50);

    // ================
    // INSERT MAIN DRIVING CODE ABOVE
    //=================
    totalTime = (millis() - startTime)/1000.0;
    Serial.println("Finished button A function with time of " + String(totalTime) + "seconds."); 
    ledGreen(false); 
  }

  // TESTING 
  if(buttonB.isPressed()){
    delay(1000);

    ledRed(true);
    findFF();
    motors.setSpeeds(22, 22);
    delay(1000);
    motors.setSpeeds(0, 0);
    ledRed(false);


  }

  if(buttonC.isPressed()){
    delay(1000);

    initMotionControl();
  }
}