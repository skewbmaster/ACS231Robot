#include "definitions.hh"
#include "PIDController.hh"
#include <Servo.h>

#define LOOP_TIME 25  // Loop should run at 40Hz
#define DEBUG

bool ForwardRobot(float distance, float cutoff);
bool BackwardRobot(float distance, float cutoff);
bool RotateRobot(float degAngle, bool counterClock = true);
float SampleSonicSensor(float time);

Servo S1;
Servo S2;

Wheels wheels;
PIDController movementControl = PIDController(0, 0, 0, 0, 0);

ServoArm* armControl;

RobotState stateMachine;

unsigned long previousMillis = 0;
unsigned long dropPenTimestamp = 0;
unsigned long moveDownArmTime = 0;
float yPosArm = DRAW_INIT_HEIGHT;
float robotGlobalAngle = 0;
float sampledDistanceFromWall = -1;

#ifdef DEBUG
int spwm = 1500;
#endif

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  SetupSonicSensor();

  wheels = Wheels();
  movementControl = PIDController(0.5, 0.0001, 0.2, 200, 28);

  S1.attach(SERVO_S1_PIN);
  S2.attach(SERVO_S2_PIN);

  yPosArm = DRAW_INIT_HEIGHT;
  armControl = new ServoArm(&S1, &S2, DRAW_X_POS, yPosArm);
  //armControl->MoveArm(DRAW_X_POS, yPosArm);
  S1.writeMicroseconds(SERVO_S1_RESTING_PWM);
  S2.writeMicroseconds(SERVO_S2_RESTING_PWM);

  stateMachine = FORWARD_3;
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < LOOP_TIME) {
    return;
  }
  previousMillis = currentMillis;

  /*if (currentMillis - moveDownArmTime > DRAW_LOOP_TIME) {
    if (yPosArm >= DRAW_END_HEIGHT) {
      yPosArm -= DRAW_INTERP_TIME * (currentMillis - moveDownArmTime);
      armControl->MoveArm(290, yPosArm);
    }
    moveDownArmTime = currentMillis;
  }*/

  //int spd = Serial.parseInt() - 128;
  /*float distTravelled = wheels.GetWheelMovedDistance();
  float error = distToTravel - distTravelled;
  if (error > 4) {
    float motorPower = movementControl.Output(error);
    wheels.Drive((int) motorPower);
  }
  else {
    wheels.Brake();
  }
  Serial.println(distTravelled);*/

  //int _i = LineFollowState();
  //Serial.println(_i);

  //RotateRobot(180);

  /*int read = Serial.parseInt();
  if (read != 0) {
    spwm = read;
  }

  S2.writeMicroseconds(spwm);*/
  //S2.writeMicroseconds(spwm);*/



  switch (stateMachine) {
    case INIT_ROTATE_1: {
      if (RotateRobot(180)) {
        stateMachine = INIT_ROTATE_2;
        delay(1500);
        wheels.ResetEncoderCounts();
      }
      break;
    }

    case INIT_ROTATE_2: {
      if (RotateRobot(180)) {
        stateMachine = FORWARD_1;
        delay(200);
        wheels.ResetEncoderCounts();
      }
      break;
    }

    case FORWARD_1: {
      if (ForwardRobot(650, 5)) {
        stateMachine = ROTATE_1;
        delay(150);
        wheels.ResetEncoderCounts();
        movementControl.ResetIntegral();
      }
      break;
    }

    case ROTATE_1: {
      if (RotateRobot(90)) {
        stateMachine = FORWARD_INDEFINITE;
        delay(150);
        wheels.ResetEncoderCounts();
      }
      break;
    }

    case FORWARD_INDEFINITE: {
      wheels.Drive(130);
      PollDistance();
      float readDistance = GetDistance();
      if (readDistance > 0 && readDistance < 80) {
        wheels.Brake();
        stateMachine = BACKWARDS_WALL;
        delay(400);
        wheels.ResetEncoderCounts();
      }
      break;
    }

    case SAMPLE_SONIC: {
      sampledDistanceFromWall = SampleSonicSensor(2*1000);
      Serial.println(sampledDistanceFromWall);
      stateMachine = BACKWARDS_WALL;
      break;
    }

    case BACKWARDS_WALL: {
      if (BackwardRobot(68, 2)) {
        stateMachine = ROTATE_WALL;
        delay(150);
        wheels.ResetEncoderCounts();
        movementControl.ResetIntegral();
      }
      break;
    }

    case ROTATE_WALL: {
      armControl->MoveArm(DRAW_X_POS, yPosArm);
      if (RotateRobot(180)) {
        stateMachine = DRAW_LINE;
        delay(150);
        wheels.ResetEncoderCounts();
        moveDownArmTime = millis();
      }
      break;
    }

    case DRAW_LINE: {
      float loopTime = currentMillis - moveDownArmTime;
      if (loopTime > DRAW_LOOP_TIME && yPosArm >= DRAW_END_HEIGHT) {
        yPosArm -= DRAW_INTERP_TIME * loopTime;
        //Serial.println(yPosArm);
        armControl->MoveArm(DRAW_X_POS, yPosArm);
        moveDownArmTime = currentMillis;
      }
      if (yPosArm <= DRAW_END_HEIGHT) {
        stateMachine = RESET_ARM;
        Serial.println("finished draw!");
        delay(150);
      }
      break;
    }

    case RESET_ARM: {
      if (ForwardRobot(50, 3)) {
        S2.writeMicroseconds(SERVO_S2_RESTING_PWM);
        //armControl->MoveArm(DRAW_X_POS, DRAW_INIT_HEIGHT/6.0);
        delay(1000);
        wheels.ResetEncoderCounts();
        movementControl.ResetIntegral();
        stateMachine = ROTATE_2;
      }
      break;
    }

    case ROTATE_2: {
      if (RotateRobot(90, false)) {
        stateMachine = FORWARD_2;
        delay(150);
        wheels.ResetEncoderCounts();
      }
      break;
    }

    case FORWARD_2: {
      S1.writeMicroseconds(SERVO_S1_RESTING_PWM);
      if (ForwardRobot(650, 5)) {
        stateMachine = ROTATE_3;
        delay(150);
        wheels.ResetEncoderCounts();
        movementControl.ResetIntegral();
      }
      break;
    }

    case ROTATE_3: {
      if (RotateRobot(90)) {
        stateMachine = FORWARD_3;
        delay(150);
        wheels.ResetEncoderCounts();
      }
      break;
    }

    case FORWARD_3: {
      if (ForwardRobot(740, 5)) {
        delay(150);
        wheels.ResetEncoderCounts();
        movementControl.ResetIntegral();
        stateMachine = DROP_PEN;
        dropPenTimestamp = millis();
      }
      break;
    }

    case DROP_PEN: {
      if (currentMillis - dropPenTimestamp < 3000) {
        armControl->MoveArm(240, -10);
        delay(50);
        armControl->MoveArm(250, 0);
        delay(50);
      }
    }

    default: {
      //Serial.println("You reached a fake case !");
      break;
    }
  }
}

// Returns true when complete
bool ForwardRobot(float distance, float cutoff) {
  float avg_travelled = (abs(wheels.GetWheelAMovedDistance()) + abs(wheels.GetWheelBMovedDistance())) / 2.0;
  float error = distance - avg_travelled;
  if (abs(error) > cutoff) {
    float motorPower = movementControl.Output(error);
    wheels.Drive((int) motorPower);
    return false;
  }
  wheels.Brake();
  return true;
}

// Returns true when complete
bool BackwardRobot(float distance, float cutoff) {
  float avg_travelled = (abs(wheels.GetWheelAMovedDistance()) + abs(wheels.GetWheelBMovedDistance())) / 2.0;
  float error = distance - avg_travelled;
  if (abs(error) > cutoff) {
    float motorPower = -movementControl.Output(error);
    wheels.Drive((int) motorPower);
    return false;
  }
  wheels.Brake();
  return true;
}

// Returns true when complete
bool RotateRobot(float degAngle, bool counterClock = true) {
  float avg_travelled = (abs(wheels.GetWheelAMovedDistance()) + abs(wheels.GetWheelBMovedDistance())) / 2.0;
  //Serial.println(distTravelled);

  float distanceToTurn = COEFF_ROTATION * PI * ROBOT_DIAMETER * degAngle / 360.0;
  /*Serial.print(avg_travelled);
  Serial.print(" ");
  Serial.println(distanceToTurn);*/

  if (abs(avg_travelled) < distanceToTurn) {
    wheels.Rotate(counterClock);
    return false;
  }

  wheels.Brake();
  return true;
}

float SampleSonicSensor(float time) {
  float samples = 0;
  float distTotal;

  unsigned long currentTime = millis();
  unsigned long startTime = currentTime;
  unsigned long prevTime = currentTime;

  PollDistance();
  while ((currentTime - startTime) <= time) {
    PollDistance();
    float dist = GetDistance();
    //Serial.println(dist);
    if (dist > 0) {
      samples += 1;
      distTotal += dist;
    }
    while (prevTime)
    currentTime = millis();

  }
  return distTotal / samples;
}
