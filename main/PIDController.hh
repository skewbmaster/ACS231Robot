#pragma once

class PIDController {
  public:
    PIDController(float, float, float, float, float);
    float Output(float, float);
    void UpdateK(float, float, float);
  private:
    float Kp, Ki, Kd, maxOut, minOut;
    float dt;
    float prev_output;
};
