#include <Romi32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "PIDController.h"

extern Romi32U4Motors motors; 
extern Romi32U4Encoders encoders;
extern LSM6 gyro;

void forward(float distance_cm);
void turnRight(float angle = 90);
void turnLeft(float angle = 90);

void initMotionControl();

void resetSensors();
void encodersReset();
void PIDReset();
float getDistanceTravelled(); // in cm

void gyroInit();
void gyroReset();
float getTurnAngle();
float updateTurnAngle(); 

float inToCm(float in); // converts inches to cm function. 

// void maintainHeadingTest(); // for pid  tuning. 
void findFF(); 

