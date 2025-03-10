#include "definitions.hh"

#define LOOP_TIME 25 // Loop should run at 40Hz
#define DEBUG

//Motor::Motor(int In1Pin, int In2Pin, int PWMPin, int offset, int STBYPin)

Wheels wheels;

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  SetupSonicSensor();

  wheels = Wheels();

  //wheels.Drive(128);
}

unsigned long previousMillis = 0;

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
  float dist = wheels.GetWheelMovedDistance();
  if (abs(dist) < WHEEL_DIAMETER_MM * PI) {
    wheels.Drive(100);
  }
  else {
    wheels.Brake();
  }

  //Serial.println();

  //int _i = LineFollowState();
  //Serial.println(_i);
}
