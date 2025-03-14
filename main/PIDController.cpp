#include "PIDController.hh"
#include "Arduino.h"

PIDController::PIDController(float Kp, float Ki, float Kd, float maxBound, float minBound) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->maxOut = maxBound;
  this->minOut = minBound;

  summed_error = 0;
  prev_error = 0;
  prev_time = millis();
}

float PIDController::Output(float error) {
  float time = millis();
  float dt = time - prev_time;
  prev_time = time;

  float output = Kp * error;

  summed_error += error * dt;
  output += Ki * summed_error;

  output += Kd * ((error - prev_error) / dt);

  output = min(max(output, minOut), maxOut);
  return output;
}

void PIDController::ResetIntegral() {
  summed_error = 0.0;
}
