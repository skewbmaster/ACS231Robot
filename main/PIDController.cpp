#include "PIDController.hh"

PIDController::PIDController(float Kp, float Ki, float Kd, float maxBound, float minBound) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->maxOut = maxBound;
  this->minOut = minBound;
}

float PIDController::

float PIDController::Output(float reference, float sensor) {
  float error = reference - sensor;
}
