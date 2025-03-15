#include "Arduino.h"
#include "MotionControl.h"
#include <Romi32U4.h>
#include <Wire.h>
#include <LSM6.h>

// Constants - May need tunning   
// Gear reduction: 120:1
// Wheel diameter: 70mm
// Encoder counts per wheel rotation: 1440
const float countsPerCm = 1440.0 / (M_PI * 7.0); // ~65.48089087
const int16_t maxSpeed = 370; // must be less than motor max speed - correction speed
const int16_t maxHeadingCorrection = 30;
const int16_t maxTurnSpeed = 80;  

const int16_t maxAccel = 3; //units of motor speed (-300, 300)
// const int16_t maxDecel = 12; 
float speed = 0; 

// ==============
// PIDs may need adjustments on the spot. 
// ==============
// kP, kI, kD, kFF error deadband
PIDController drivingPID(0.2, 0, 0.1, 22, 3);
PIDController headingPID(2, 0, 0.3, 22, 0.6); // deadband units. 
PIDController headingCorrectionPID(6, 0, 0.4, 0, 0.25);

Romi32U4Motors motors; 
Romi32U4Encoders encoders;
LSM6 gyro;

const float sensitivity = 0.035; // for gyro 0.00875
float gyroOffset;
float currentCounts = 0; //encoder counts
float turnAngle = 0.0;
float turnRate;
unsigned long gyroLastUpdate = 0; 
int feedForward = 0;

void gyroInit(){
  gyro.init();
  gyro.enableDefault();
  gyro.writeReg(LSM6::CTRL2_G, 0b10001000);

  int32_t total = 0;
  for (uint16_t i = 0; i < 5000; i++)
  {
    // Wait for new data to be available, then read it.
    while(!gyro.readReg(LSM6::STATUS_REG) & 0x08);
    gyro.read();

    // Add the Z axis reading to the total.
    // Serial.println(String(gyro.g.z));
    total += gyro.g.z;
  }
  gyroOffset = (total / 5000) * sensitivity; 
  // Serial.println("OFFSET: " + String(gyroOffset));
  // gyroReset(); 
  // Serial.println("Angle: " + String(updateTurnAngle()));
}

void encodersReset(){
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void PIDSReset(){
  drivingPID.reset();
  headingPID.reset(); 
}

void gyroReset(){
  turnAngle = 0;
  gyroLastUpdate = micros();
}

void initMotionControl(){
  delay(750);
  ledYellow(true);
  Wire.begin();
  gyroInit();
  encodersReset();
  gyroReset();
  ledYellow(false);

}

float getDistanceTravelled(){
  return (currentCounts)/(countsPerCm);
}

float getTurnAngle() {
  return turnAngle; 
}

float updateTurnAngle() {
  gyro.readGyro(); 
  turnRate = gyro.g.z * sensitivity - gyroOffset; // degrees per second, +/- ~1 dps
  // Serial.println(String(turnRate)); 
  unsigned long m = micros();
  float dt = (m - gyroLastUpdate) / 1000000.0; // seconds
  gyroLastUpdate = m; 

  turnAngle += turnRate * dt;
  return turnAngle;  
}

void forward(float distance_cm){
  currentCounts = 0;
  delay(100); 
  motors.allowTurbo(true); 
  encodersReset();
  gyroReset();
  PIDSReset(); 
  float targetCounts = distance_cm * countsPerCm; 
  float targetHeading = updateTurnAngle(); 
  float accumulatedCounts = 0; // encoders record max 32k counts; reset at 30k and record. 
  while(true){
    float leftCounts = encoders.getCountsAndResetLeft();
    float rightCounts = encoders.getCountsAndResetRight();
    currentCounts = currentCounts + (leftCounts + rightCounts) / 2;  
    float distanceError = targetCounts - currentCounts; 
    // Serial.println("left counts: " + String(leftCounts));
    // Serial.println("right counts: " + String(rightCounts));
    // Serial.println("Current counts: " + String(currentCounts) + "; Target Counts: " + String(targetCounts) + "; Count Error: " + String(distanceError));
    float drivePower = drivingPID.calculate(distanceError); 
    drivePower = constrain(drivePower, -maxSpeed, maxSpeed); 
    if(drivePower >= 0){
      drivePower = constrain(drivePower, -maxSpeed, speed+maxAccel); 
    } else {
      drivePower = constrain(drivePower, speed-maxAccel, maxSpeed); 
    }
    speed = drivePower; 

    float currentHeading = updateTurnAngle();
    float headingError = targetHeading - currentHeading;
    float headingCorrection = constrain(headingCorrectionPID.calculate(headingError), -maxHeadingCorrection, maxHeadingCorrection); 
    if(drivePower == 0){ 
      motors.setSpeeds(0, 0);
      speed = 0;
      Serial.println("Forward Drive Done. Final error: " + String(distanceError/countsPerCm) + "cm. Final heading: " + String(turnAngle) + "degrees.");
      break; 
    }
    Serial.println(drivePower);
    motors.setSpeeds(drivePower - headingCorrection, drivePower + headingCorrection); 
    delay(5); 

  }
}

void turnLeft(float angle = 90){
  delay(100); 
  encodersReset();
  gyroReset();
  PIDSReset(); 
  while(true){
    float currentHeading = updateTurnAngle();
    Serial.println(String(currentHeading)); 
    float headingError = (angle) - currentHeading; 
    float headingCorrection = constrain(headingPID.calculate(headingError), -maxTurnSpeed, maxTurnSpeed); 
    if(headingCorrection == 0){
      motors.setSpeeds(0,0);
      Serial.println("Turn done. Final Turn Angle: " + String(turnAngle)); 
      break;
    }
    motors.setSpeeds(-headingCorrection, headingCorrection);
    delay(5); 
  }
}

void turnRight(float angle = 90){
  delay(100); 
  turnLeft(-angle); 
}

float inToCm(float inches){
  return inches * 2.54; 
}

void findFF(){
  feedForward = 0; 
  float distanceTravelled = 0; 
  while(true){
    motors.setSpeeds(feedForward, feedForward);
    Serial.println("FeedForward: " + String(feedForward)); 
    delay(20); 
    distanceTravelled += (encoders.getCountsAndResetLeft() + encoders.getCountsAndResetRight())/(2*countsPerCm);
    if(distanceTravelled > 0.05){
      Serial.println("Final FeedForward: " + String(feedForward));
      motors.setSpeeds(0, 0); 
      break; 
    } else{
      feedForward += 1; 
    }
  }
}

// void maintainHeadingTest(){
//   encodersReset();
//   gyroReset();
//   PIDSReset();

//   float targetHeading = updateTurnAngle(); 

//   while(true){
//     float headingError = targetHeading - updateTurnAngle(); 
//     float headingCorrection = constrain(headingPID.calculate(headingError), -maxHeadingCorrection, maxHeadingCorrection);
//     motors.setSpeeds(-headingCorrection, headingCorrection); 
//     // gyro.readGyro();
//     // Serial.println(turnAngle);
//     delay(15);

//   }
// }