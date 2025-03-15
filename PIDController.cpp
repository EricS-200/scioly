#include "PIDController.h"
#include <Wire.h>
#include <Romi32U4.h>

PIDController::PIDController(float kp, float ki, float kd, float kff, float db) : kP(kp), kI(ki), kD(kd), kFF(kff), integral(0), lastError(0), deadband(db) {}

float PIDController::calculate(float error){
  // Serial.println("PID Error: " + String(error));

  if (abs(error) < deadband) {
    return 0;
  }

  integral += error;

  float derivative = error - lastError; 

  lastError = error; 

  float output = kP * error + kI * integral + kD * derivative; 

  if (output > 0 && output < kFF) {
      output = kFF;
  } else if (output < 0 && output > -kFF) {
      output = -kFF; 
  }
  return output; 
}

void PIDController::reset(){
  integral = 0;
  lastError = 0; 
}