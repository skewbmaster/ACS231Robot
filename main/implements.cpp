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
  }
  else {
    digitalWrite(MOTOR_A_I1, LOW);
    digitalWrite(MOTOR_A_I2, HIGH);
  }

  analogWrite(MOTOR_A_PWM_PIN, abs(speed));

}

void Wheels::Brake() {
  digitalWrite(MOTOR_A_I1, HIGH);
  digitalWrite(MOTOR_A_I2, HIGH);
  analogWrite(MOTOR_A_PWM_PIN, 150);
}

float Wheels::GetWheelMovedDistance() {
  return WHEEL_DIAMETER_MM * PI * encA_Count / MOTOR_PULSES;
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


  /*Serial.print(leftDark);
  Serial.print(" ");
  Serial.print(centreDark);
  Serial.print(" ");
  Serial.println(rightDark);*/
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
