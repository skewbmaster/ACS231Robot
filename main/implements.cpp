#include "definitions.hh"
#include "Arduino.h"

volatile signed long encA_Count;
volatile signed long encB_Count;
Wheels::Wheels() {
  
  pinMode(MOTOR_A_I1, INPUT);
  pinMode(MOTOR_A_I2, INPUT);
  pinMode(MOTOR_B_I1, INPUT);
  pinMode(MOTOR_B_I2, INPUT);
  pinMode(MOTOR_STBY_PIN, INPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCA_PIN), MotorA_ChannelA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCB_PIN), MotorA_ChannelB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENCA_PIN), MotorB_ChannelA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENCB_PIN), MotorB_ChannelB_ISR, CHANGE);

  encA_Count = 0;
  encB_Count = 0;
  wheelAngle = 0;

  digitalWrite(MOTOR_STBY_PIN, HIGH);
}

void Wheels::Drive(int speed) {
  //speed = min(max(speed, -250), 250);
  if (speed > 0) {
    digitalWrite(MOTOR_A_I1, HIGH);
    digitalWrite(MOTOR_A_I2, LOW);
    digitalWrite(MOTOR_B_I1, LOW);
    digitalWrite(MOTOR_B_I2, HIGH);
  }
  else {
    digitalWrite(MOTOR_A_I1, LOW);
    digitalWrite(MOTOR_A_I2, HIGH);
    digitalWrite(MOTOR_B_I1, HIGH);
    digitalWrite(MOTOR_B_I2, LOW);
  }

  analogWrite(MOTOR_A_PWM_PIN, abs(speed));
  analogWrite(MOTOR_B_PWM_PIN, abs(speed));

}

void Wheels::Rotate(bool counterClock = true) {
  if (counterClock) {
    digitalWrite(MOTOR_A_I1, LOW);
    digitalWrite(MOTOR_A_I2, HIGH);
    digitalWrite(MOTOR_B_I1, LOW);
    digitalWrite(MOTOR_B_I2, HIGH);
  }
  else {
    digitalWrite(MOTOR_A_I1, HIGH);
    digitalWrite(MOTOR_A_I2, LOW);
    digitalWrite(MOTOR_B_I1, HIGH);
    digitalWrite(MOTOR_B_I2, LOW);
  }

  analogWrite(MOTOR_A_PWM_PIN, 100);
  analogWrite(MOTOR_B_PWM_PIN, 100);
}

void Wheels::Brake() {
  digitalWrite(MOTOR_A_I1, HIGH);
  digitalWrite(MOTOR_A_I2, HIGH);
  digitalWrite(MOTOR_B_I1, HIGH);
  digitalWrite(MOTOR_B_I2, HIGH);
  analogWrite(MOTOR_A_PWM_PIN, 150);
  analogWrite(MOTOR_B_PWM_PIN, 150);
}

float Wheels::GetWheelAMovedDistance() {
  return COEFF_TRACTION * WHEEL_DIAMETER_MM * PI * encA_Count / MOTOR_PULSES;
}
float Wheels::GetWheelBMovedDistance() {
  return COEFF_TRACTION * WHEEL_DIAMETER_MM * PI * encB_Count / MOTOR_PULSES;
}

void Wheels::ResetEncoderCounts() {
  encA_Count = 0;
  encB_Count = 0;
}

ServoArm::ServoArm(Servo* Servo1, Servo* Servo2, float defaultPosX, float defaultPosY) {
  posX = defaultPosX;
  posY = defaultPosY;

  L1_2 = SERVO_ARM_L1 * SERVO_ARM_L1;
  L2_2 = SERVO_ARM_L2 * SERVO_ARM_L2;
  L1_L2 = SERVO_ARM_L1 * SERVO_ARM_L2;

  S1 = Servo1;
  S2 = Servo2;
}

void ServoArm::MoveArm(float newPosX, float newPosY) {
  posX = newPosX;
  posY = newPosY;
  S1->writeMicroseconds(1500);
  S2->writeMicroseconds(1500);
}

ServoArm::getTheta1() {
  float x_2 = posX * posX;
  float y_2 = posY * posY;

  float sigma = 2 * (L1_L2*L1_L2 + L1_2*x_2 + L1_2*y_2 + L2_2*x_2 + L2_2*y_2 - x_2*y_2);
  sigma -= L1_2*L1_2 + L2_2*L2_2 + x_2*x_2 + y_2*y_2;
  sigma = sqrt(sigma);

  float theta = 2 * atan2(2*SERVO_ARM_L1*posY + sigma,
                           L1_2 + 2*SERVO_ARM_L1*posX - L2_2 + x_2 + y_2);

  return theta;
}

ServoArm::getTheta2() {
  float x_2 = posX * posX;
  float y_2 = posY * posY;
  
  float part1 = 2*L1_L2 + x_2 + y_2 - L1_2 - L2_2;
  float part2 = 2*L1_L2 + L1_2 + L2_2 - x_2 - y_2;
  float theta = -2 * atan2(sqrt(part1*part2), part1);
  return theta;
}

volatile bool echoInProgress;
static unsigned long risingTimestamp;
volatile float lastReadDistance;
void SetupSonicSensor() {
  pinMode(SONIC_TRIG_PIN, OUTPUT);
  pinMode(SONIC_ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(SONIC_ECHO_PIN), EchoISR, CHANGE);
  echoInProgress = false;
  risingTimestamp = 0;
  lastReadDistance = 0;
}

void PollDistance() {
  digitalWrite(SONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONIC_TRIG_PIN, LOW);
}

float GetDistance() {
  if (echoInProgress) {
    return -1.0;
  }
  return lastReadDistance;
}

void EchoISR() {
  unsigned long currentMicros = micros();
  int echoState = digitalRead(SONIC_ECHO_PIN);

  if (echoState == HIGH) {
    risingTimestamp = currentMicros;
    echoInProgress = true;
  }
  else {
    unsigned int pulseTime = currentMicros - risingTimestamp;
    echoInProgress = false;
    lastReadDistance = pulseTime * SOUND_MM_PER_US / 2.0;
  }
}

int LineFollowState() {
  int leftV = analogRead(LINE_LEFT_PIN);
  int centreV = analogRead(LINE_CENTRE_PIN);
  int rightV = analogRead(LINE_RIGHT_PIN);

  bool leftDark = leftV > LINE_LEFT_THLD;
  bool centreDark = centreV > LINE_CENTRE_THLD;
  bool rightDark = rightV > LINE_RIGHT_THLD;


  Serial.print(leftV);
  Serial.print(" ");
  Serial.print(centreV);
  Serial.print(" ");
  Serial.println(rightV);
  return 0b100*leftDark + 0b10*centreDark + 0b1*rightDark;
}

void MotorA_ChannelA_ISR() {
  int chanA = digitalRead(MOTOR_A_ENCA_PIN);
  int chanB = digitalRead(MOTOR_A_ENCB_PIN);

  if (chanA == chanB) {
    encA_Count--;    return;
  }
  encA_Count++;
}

void MotorA_ChannelB_ISR() {
  int chanA = digitalRead(MOTOR_A_ENCA_PIN);
  int chanB = digitalRead(MOTOR_A_ENCB_PIN);

  if (chanA == chanB) {
    encA_Count++;
    return;
  }
  encA_Count--;
}

void MotorB_ChannelA_ISR() {
  int chanA = digitalRead(MOTOR_B_ENCA_PIN);
  int chanB = digitalRead(MOTOR_B_ENCB_PIN);

  if (chanA == chanB) {
    encB_Count++;
    return;
  }
  encB_Count--;
}

void MotorB_ChannelB_ISR() {
  int chanA = digitalRead(MOTOR_B_ENCA_PIN);
  int chanB = digitalRead(MOTOR_B_ENCB_PIN);

  if (chanA == chanB) {
    encB_Count--;
    return;
  }
  encB_Count++;
}
