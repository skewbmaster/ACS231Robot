#pragma once

class PIDController {
  public:
    PIDController(float Kp, float Ki, float Kd, float maxBound, float minBound);
    float Output(float error);
    void UpdateK(float Kp, float Ki, float Kd);
  private:
    float Kp, Ki, Kd, maxOut, minOut;
    float summed_error;
    float prev_error, prev_output, prev_time;
};
