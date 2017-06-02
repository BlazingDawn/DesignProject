#include <Servo.h>
//#include <SabertoothSimplified.h>
#include <SoftwareSerial.h>
//#include <Streaming.h>
#define scissor 1
#define belt 2

Servo frmotor;
Servo brmotor; 
Servo flmotor; 
Servo blmotor; 

int leftMaxFront = 120; 
int rightMaxFront = 60;

int leftMaxBack = 60; 
int rightMaxBack = 120; 

int leftSpeed = 90; 
int rightSpeed = 90;

int leftramp = 5; //goes from 90 to 180 
int rightRamp = -5;//Goes from 90 to 0
int state = 0; 
/*
 * State:
 * 1 -> Forward 
 * 2 -> Backward 
 * 3 -> Left 
 * 4 -> Right
 */

int scissorspeed = 0;
int beltspeed = 0;
const int limit1 = 29;
const int limit2 = 30;
int switch1 = 0;
int switch2 = 0;

//SabertoothSimplified Drive(Serial2);
SoftwareSerial Operations(NOT_A_PIN,7);


void setup()  {
  frmotor.attach(2);
  brmotor.attach(5);
  flmotor.attach(3); 
  blmotor.attach(6);
  
  stopMotor();
  Serial.begin(9600);
  Serial1.begin(57600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  Operations.begin(9600);
  scissorspeed = 0;
  beltspeed = 0;
//  Drive.motor(scissor,scissorspeed);
//  Drive.motor(belt,beltspeed);
  //stopMotor();
}

void loop()
{
  char in;
  if (Serial.available()) {
    in = Serial.read();
    Serial.print(in);
    commands(in);
  }
  switch1 = digitalRead(limit1);
  switch2 = digitalRead(limit2);
//  if (switch1 == 1){
////    Drive.motor(scissor,50);
//    delay(50);
////    Drive.motor(scissor,0);
//  }
//  if (switch2 == 1){
////    Drive.motor(scissor,-50);
//    delay(50);
////    Drive.motor(scissor,0);
//  }
}

void commands(char in)
{
  switch (in)
  {
    case 'w':
      fwdMotor();
      break;
      //Back right goes back

    case 's':
      bckMotor();
      break;
    case 'a': 
      turnLeft(); 
    case 'd': 
      turnRight();

    case 'b':
      stopMotor();
      break;
      
    case 'u':
      scissorUp();
      break;

    case 'i':
      scissorDown();
      break;

    case 'j':
      pulleyOn();
      break;
      
    case 'k':
      pulleyOff();
      break;
    case 'l': 
      dig(); 
      break;
  }
}

void fwdMotor() {
  if(state != 1) {
    stopMotor();
    state = 1;
  }
  
 
//   rightSpeed goes from 90 to 0 
//  leftSpeed goes from 90 to 180
//  rightSpeed = (rightSpeed >= rightMaxFront) ? rightSpeed + rightRamp : rightSpeed; 
//  leftSpeed = (leftSpeed <= leftMaxFront) ? leftSpeed + leftramp : leftSpeed; 
//  driveCommand(rightSpeed, leftSpeed);
  
/*  frmotor.write(70); 
  brmotor.write(70);
  flmotor.write(110); //forward 
  blmotor.write(110); //forstesint
  */
  Serial.print("F");
}

void bckMotor() {
  if(state != 2) stopMotor();
  state = 2;

  //backward Motion
  //rightSpeed goes from 90 to 180 
  //leftSpeed goes from 90 to 0
  if(state != 2) {
    stopMotor(); 
    state = 2; 
  }
  
//  rightSpeed = (rightSpeed <= rightMaxBack) ? rightSpeed - rightRamp : rightSpeed; 
//  leftSpeed = (leftSpeed >= leftMaxBack) ? leftSpeed - leftramp : leftSpeed; 
//  driveCommand(rightSpeed, leftSpeed);
  
  
  frmotor.write(110); 
  brmotor.write(110);
  flmotor.write(70); //back
  blmotor.write(70); //for testing

  Serial.print("B");
}

void turnLeft(){ 
  //Forward motion: 
  //rightSpeed goes from 90 to 0 
  //leftSpeed goes from 90 to 180

  //backward Motion
  //rightSpeed goes from 90 to 180 
  //leftSpeed goes from 90 to 0
  
  //When turning left, left side goes backward(CW), right side goes forward(CW)
  if(state != 3) stopMotor(); 
  state = 3; 
  
  rightSpeed = (rightSpeed >= rightMaxFront) ? rightSpeed - 2: rightSpeed;
  leftSpeed = (leftSpeed >= leftMaxBack) ? leftSpeed - 2 : leftSpeed; //less ramp than usual because turning is scaaryyy
  
  driveCommand(rightSpeed, leftSpeed);
  
  Serial.print("a");
 
//  frmotor.write(70); 
//  brmotor.write(70);
//  flmotor.write(70); 
//  blmotor.write(70);
}
void turnRight(){ 
  //Forward motion: 
  //rightSpeed goes from 90 to 0 (CW)
  //leftSpeed goes from 90 to 180 (CCW)

  //backward Motionse
  //rightSpeed goes from 90 to 180 (CCW)
  //leftSpeed goes from 90 to 0 (CW)
  
  //When turning right, left side goes forward(CCW), right side goes backward(CCW)
  if(state != 4) stopMotor(); 
  state = 4; 
  
  rightSpeed = (rightSpeed <= rightMaxBack) ? rightSpeed + 2: rightSpeed;
  leftSpeed = (leftSpeed <= leftMaxFront) ? leftSpeed + 2 : leftSpeed; //less ramp than usual because turning is scaaryyy
  driveCommand(rightSpeed, leftSpeed);
  
  Serial.print("d");
 
//  frmotor.write(110); 
//  brmotor.write(110);
//  flmotor.write(110); 
//  blmotor.write(110);
}
void driveCommand(int left, int right){
  frmotor.write(right); 
  brmotor.write(right);
  flmotor.write(left); 
  blmotor.write(left);
}

void stopMotor() {
  state = 0; 
  frmotor.write(90);
  brmotor.write(90);
  flmotor.write(90);
  blmotor.write(90);

  leftSpeed = 90; 
  rightSpeed = 90; 
  Serial2.write(64);
  Serial2.write(192);
//  Drive.motor(scissor,0);
//  Drive.motor(belt,0);
}

void scissorUp() {
  scissorspeed = (scissorspeed > -100 ? (scissorspeed-25) : (scissorspeed+0));
  //Drive.motor(scissor,scissorspeed);

  Serial2.write(127); 
}

void scissorDown() {
  scissorspeed = (scissorspeed > 100 ? (scissorspeed+25) : (scissorspeed+0));
  //Drive.motor(scissor,scissorspeed);
  Serial2.write(1); 
}

void pulleyOn() {
  beltspeed = (beltspeed < 120 ? (beltspeed+60) : (beltspeed+0));
//  Drive.motor(belt,beltspeed);
  Serial2.write(128); 
}

void pulleyOff() {
//  Drive.motor(belt,0);
  Serial2.write(255); 
}

void dig(){
  frmotor.write(70); 
  brmotor.write(110);
  flmotor.write(110); //back
  blmotor.write(70); //for testing
}

