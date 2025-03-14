#include "definitions.hh"
#include "PIDController.hh"
#include <Servo.h>

#define LOOP_TIME 30  // Loop should run at 40Hz
#define DEBUG

bool ForwardRobot(float distance, float cutoff);
bool BackwardRobot(float distance, float cutoff);
bool RotateRobot(float degAngle, bool counterClock = true);

Servo S1;
Servo S2;

Wheels wheels;
PIDController movementControl = PIDController(0, 0, 0, 0, 0);

RobotState stateMachine;

unsigned long previousMillis = 0;
float distToTravel = 650;
float robotGlobalAngle = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  SetupSonicSensor();

  wheels = Wheels();
  movementControl = PIDController(0.7, 0.0001, 0.2, 200, 28);

  //S1.attach(SERVO_S1_PIN);
  //S2.attach(SERVO_S2_PIN);

  stateMachine = INIT_ROTATE_1;
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < LOOP_TIME) {
    return;
  }
  previousMillis = currentMillis;

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
  Serial.println(spwm);

  S1.writeMicroseconds(700);
  S2.writeMicroseconds(spwm);*/

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
      if (readDistance > 0 && readDistance < 100) {
        wheels.Brake();
        stateMachine = SAMPLE_SONIC;
        delay(150);
        wheels.ResetEncoderCounts();
        readDistance = GetDistance();
      }
      break;
    }

    case SAMPLE_SONIC: {
      if (true) {
        stateMachine = BACKWARDS_WALL;
        delay(150);
      }
      break;
    }

    case BACKWARDS_WALL: {
      if (BackwardRobot(85, 2)) {
        stateMachine = ROTATE_WALL;
        delay(150);
        wheels.ResetEncoderCounts();
        movementControl.ResetIntegral();
      }
      break;
    }

    case ROTATE_WALL: {
      if (RotateRobot(180)) {
        stateMachine = DRAW_LINE;
        delay(150);
        wheels.ResetEncoderCounts();
      }
      break;
    }

    case DRAW_LINE: {
      stateMachine = ROTATE_2;
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
      if (ForwardRobot(700, 5)) {
        stateMachine = DROP_PEN;
        delay(150);
        wheels.ResetEncoderCounts();
        movementControl.ResetIntegral();
      }
      break;
    }

    default: {
      Serial.println("You reached a fake case !");
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
  Serial.println("Moving back!");
  if (abs(error) > cutoff) {
    float motorPower = -movementControl.Output(error);
    Serial.println(motorPower);
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
