#include "definitions.hh"
#include "PIDController.hh"

#define LOOP_TIME 30 // Loop should run at 40Hz
#define DEBUG

//Motor::Motor(int In1Pin, int In2Pin, int PWMPin, int offset, int STBYPin)

Wheels wheels;
PIDController movementControl = PIDController(0, 0, 0, 0, 0);

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  SetupSonicSensor();

  wheels = Wheels();
  movementControl = PIDController(0.7, 0.0001, 0.1, 200, 28);



  //wheels.Drive(128);
}

unsigned long previousMillis = 0;

float distToTravel = 400;

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < LOOP_TIME) {
    return;
  }
  previousMillis = currentMillis;

  PollDistance();
  float readDistance = GetDistance();
  if (readDistance > 0) {
    //Serial.println(readDistance);
  }

  //int spd = Serial.parseInt() - 128;
  float distTravelled = wheels.GetWheelMovedDistance();
  float error = distToTravel - distTravelled;
  if (error > 4) {
    float motorPower = movementControl.Output(error);
    wheels.Drive((int) motorPower);
  }
  else {
    wheels.Brake();
  }
  Serial.println(distTravelled);

  //int _i = LineFollowState();
  //Serial.println(_i);

}
