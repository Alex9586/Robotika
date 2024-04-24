#include <AFMotor.h>

#define IR_SENSOR_RIGHT A5
#define IR_SENSOR_LEFT A4
#define TRIGGER_PIN A0
#define ECHO_PIN A2
#define MOTOR_SPEED 180

AF_DCMotor motor1(1, MOTOR12_8KHZ);  // Left motor
AF_DCMotor motor3(3, MOTOR34_8KHZ);  // Right motor

void setup()
{
  
  TCCR0B = TCCR0B & B11111000 | B00000010;
  
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //rotateMotor(0,0);   
  motor1.setSpeed(MOTOR_SPEED);
  motor3.setSpeed(MOTOR_SPEED);

  motor1.run(RELEASE);
  motor3.run(RELEASE);
}


void loop()
{

  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  float distance = measureDistance();


  if (distance <= 20){
    rotateMotor(0, 0);
  }

  // If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    motor1.run(FORWARD);
    motor3.run(FORWARD);
  }
  // If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW)
  {
    motor1.run(FORWARD);
    motor3.run(BACKWARD);
  }
  // If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH)
  {
    motor1.run(BACKWARD);
    motor3.run(FORWARD);
  }
  // If both the sensors detect black line, then stop 
  else 
  {
    motor1.run(RELEASE);
    motor3.run(RELEASE);
  }
}

/*void rotateMotor(int leftMotorSpeed, int rightMotorSpeed){
  // Set the direction and speed for the right motor (motor3)
  if (rightMotorSpeed < 0)
  {
    motor3.run(BACKWARD);
  }
  else if (rightMotorSpeed > 0)
  {
    motor3.run(FORWARD);
  }
  else
  {
    motor3.run(RELEASE);
  }

  // Set the direction and speed for the left motor (motor1)
  if (leftMotorSpeed < 0)
  {
    motor1.run(BACKWARD);
  }
  else if (leftMotorSpeed > 0)
  {
    motor1.run(FORWARD);
  }
  else 
  {
    motor1.run(RELEASE);
  }

  // Set the speed for the motors
  motor3.setSpeed(abs(rightMotorSpeed));
  motor1.setSpeed(abs(leftMotorSpeed));
}*/

float measureDistance(){
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(TRIGGER_PIN, LOW);
  
  float duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.0344 / 2; 
  
  return distance;
}