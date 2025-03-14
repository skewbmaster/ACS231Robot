#ifndef ROBOT_DEFINITIONS_H
#define ROBOT_DEFINITIONS_H
// PWM OUT

#define MOTOR_A_PWM_PIN 6
#define MOTOR_B_PWM_PIN 7

// Digital OUT
#define MOTOR_A_I1 24
#define MOTOR_A_I2 25
#define MOTOR_B_I1 26
#define MOTOR_B_I2 27
#define MOTOR_STBY_PIN 22
#define SONIC_TRIG_PIN 13

// Digital IN
// Interrupt Specific
#define SONIC_ECHO_PIN 19
#define MOTOR_A_ENCA_PIN 3
#define MOTOR_A_ENCB_PIN 18
#define MOTOR_B_ENCA_PIN 20
#define MOTOR_B_ENCB_PIN 21

// Analog IN
#define LINE_LEFT_PIN 0
#define LINE_CENTRE_PIN 1
#define LINE_RIGHT_PIN 2

#define MOTOR_PULSES 3840.0f
#define WHEEL_DIAMETER_MM 69
#define COEFF_TRACTION 1.0

#define SOUND_MM_PER_US 0.343

#define LINE_LEFT_THLD 420
#define LINE_CENTRE_THLD 250
#define LINE_RIGHT_THLD 420

class Wheels {
  public:
    Wheels();
    void Drive(int);
    void Brake();
    float GetWheelMovedDistance();
  private:
    float wheelAngle;

    void SetupMotor(int In1Pin, int In2Pin, int PWMPin, int offset, int STBYPin);
};

void MotorA_ChannelA_ISR();
void MotorA_ChannelB_ISR();
void MotorB_ChannelA_ISR();
void MotorB_ChannelB_ISR();

// Run in setup once
void SetupSonicSensor();
// Must be called before GetDistance can return a true value
void PollDistance();
// Returns the distance between ultrasonic sensor and object it's pointing to in millimeters
float GetDistance();
// Do not call in code
void EchoISR();

int LineFollowState();

#endif
