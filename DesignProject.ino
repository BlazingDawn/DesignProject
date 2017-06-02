#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Filter.h>
#include <MegunoLink.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float elapsedTime, time, timePrev;
const int analogIn1 = A5;
const int analogIn2 = A1;
const int EmergencyGnd = 4;
const int ChargOn = 5;
int trigger, RawValue1, RawValue2, tempspeed;
int state = 0;
float Amps1, Amps2;
float tempamps1, tempamps2;
int timer1 = 0;
float posx, posy, posz, accelx, accelz, accely;
ExponentialFilter<long> ADCFilter(5, 0);
Servo motor1; //Motors 1,2,3 are tri
Servo motor2;
Servo motor3;
Servo motor4; //Motors 4,5,6 are tri
Servo motor5;
Servo motor6;
float PIDm, error, prevError1, prevError2, pGain, iGain, dGain;
double point1, point2, point3, point4, point5, point6;
double ramp = 4;
double Kp= 1, Ki=0.05, Kd=1.5;

void setup() {
  time = millis();
  pinMode(EmergencyGnd, INPUT);
  digitalWrite(EmergencyGnd, HIGH);
  motor1.attach(12);  
  motor2.attach(11);  
  motor3.attach(10);  
  motor4.attach(9);  
  motor5.attach(8);  
  motor6.attach(7);
  Serial.begin(9600);
  state = 1;
  motor1.writeMicroseconds(1200); // Initialize motors
  motor2.writeMicroseconds(1200); // Turn on Arduino then battery power
  motor3.writeMicroseconds(1200);
  motor4.writeMicroseconds(1200);
  motor5.writeMicroseconds(1200);
  motor6.writeMicroseconds(1200);
  Serial.println("Power");
    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
 
  }
  delay(1000);
  Serial.println("Ready");
  motor1.writeMicroseconds(0);
  motor2.writeMicroseconds(0);
  motor3.writeMicroseconds(0);
  motor4.writeMicroseconds(0);
  motor5.writeMicroseconds(0);
  motor6.writeMicroseconds(0);
  delay(8000); //8000 in real test
  bno.setExtCrystalUse(true);
}

void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time-timePrev)/1000;
  
  sensors_event_t event; 
  bno.getEvent(&event);
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  RawValue1 = analogRead(analogIn1);
  Amps1 = 0.4317*((RawValue1/3.3)-(302/3));
  /*RawValue2 = analogRead(analogIn2);
  Amps2 = 0.4317*((RawValue2/3.3)-(302/3));*/
  digitalWrite(ChargOn, HIGH);
/*  Serial.println(point1);
  Serial.println(point2);
  Serial.println(point3);
  Serial.println(point4);
  Serial.println(point5);*/
  Serial.println(point6);
    if(state == 1){
      stateOne();
    }
    else if(state == 2){
      stateTwo();
    }
    else if (state == 3){
      stateThree();
    }
    else if(state == 5){
      stateFive();
    }
    else{
      emergency();
    }
}

void GetData(){ //Obtains gyroscope and current data
  sensors_event_t event;  // BNO055 Gyro
  bno.getEvent(&event);
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  tempamps1 = Amps1;    // Current Sensor
  RawValue1 = analogRead(analogIn1);
  Amps1 = 0.4317*((RawValue1/3.3)-(302/3));
  /*tempamps2 = Amps2;
  RawValue2 = analogRead(analogIn2);
  Amps2 = 0.4317*((RawValue2/3.3)-(302/3));*/
  ADCFilter.Filter(RawValue1);
  Serial.print("Motor Current");
  Serial.print("\t");
  Serial.println(Amps1);
  posx = event.orientation.x;
  posy = event.orientation.y-.8;
  posz = event.orientation.z+8;
  ADCFilter.Filter(posx);
  ADCFilter.Filter(posy);
  ADCFilter.Filter(posz);
  accelx = accelerometer.x();
  accely = accelerometer.y();
  accelz = accelerometer.z()-9.74;
  ADCFilter.Filter(accelz);
  ADCFilter.Filter(accely);
  ADCFilter.Filter(accelx);
}

void PIDcon(float pos, float prevError){ // PID control using the gyroscope
  if(pos>300){
    pos = pos-360;
  }
  error = pos;
  pGain = Kp*pos;
  if(-3<error<3){
  iGain = iGain+(Ki*pos);
  }
  dGain = Kd*((pos-prevError)/elapsedTime);
  PIDm = pGain+iGain+dGain;
}

void stateOne(){ //Startup
  Serial.println("Startup");
  trigger = digitalRead(EmergencyGnd);
  Serial.println(trigger);
  GetData();
  eStop();
  point1 = 1170; // Minimum throttle is 1150
  point2 = point1; // Max throttle is 2000
  point3 = point1;
  point4 = 1170;
  point5 = point4;
  point6 = 1170; 
  
  inputSpeed();
  
  Serial.println(point6);
  state = 2;
  }

void stateTwo(){ //Increase Height
    point1 = point1+ramp;
    point2 = point1;
    point3 = point1;
    point4 = point4+ramp;
    point5 = point4;
    point6 = point5;
    
  Serial.println("Up");
  trigger = digitalRead(EmergencyGnd);
  GetData();
  eStop();

  PIDcon(posz,prevError1);
  point5 = point5+PIDm;
  point3 = point5;
  point6 = point6-PIDm;
  point2 = point6;
  prevError1 = error;
  PIDcon(posy,prevError2);
  point1 = point1+PIDm;
  point4 = point4-PIDm;
  prevError2 = error;

  motorSafety();

    if(point6>1250){
      state = 3;
    }
    
  inputSpeed();
  Serial.println(point6);
  Serial.println(trigger);
  tempspeed = point6;
}
void stateThree(){ //Hover / death mode
  GetData();
  eStop();
  
  PIDcon(posz,prevError1);
  point5 = point5+PIDm;
  point3 = point5;
  point6 = point6-PIDm;
  point2 = point6;
  prevError1 = error;
  PIDcon(posy,prevError2);
  point1 = point1+PIDm;
  point4 = point4-PIDm;
  prevError2 = error;

  motorSafety();
  
  inputSpeed();

  timer1 = timer1 + 1;
      if(timer1 > 150){  Serial.println(point6);
  Serial.print("\t");
  Serial.print(point5);
  Serial.print("\t");
  Serial.print(point4);
  Serial.print("\t");
  Serial.print(point3);
  Serial.print("\t");
  Serial.print(point2);
  Serial.print("\t");
  Serial.print(point1);
        state = 4;
      }
}
void stateFour(){ //Lower Height
  point1 = point1 - 2*ramp;
  point2 = point1; point3 = point1;
  point4 = point4 - 2*ramp;
  point5 = point4; point6 = point4;

  inputSpeed();
    if(point6<1200){
      state = 5;
    }
}

void stateFive(){ // Stop
  point1 = 1000;
  point2 = 1000;
  point3 = 1000;
  point4 = 1000;
  point5 = 1000;
  point6 = 1000; 
  digitalWrite(ChargOn, HIGH);
  inputSpeed();
}


void emergency(){ //Emergency stop
  Serial.println("Emergency Stop");
  if(point1>900){
  point1 = point1-5;
  }
  point2 = point1;
  point3 = point1;
  point4 = point1;
  point5 = point1;
  point6 = point1;
  inputSpeed();
  state = 9;
}

void inputSpeed(){
  motor1.writeMicroseconds(point1);
  motor2.writeMicroseconds(point2);
  motor3.writeMicroseconds(point3);
  motor4.writeMicroseconds(point4);
  motor5.writeMicroseconds(point5);
  motor6.writeMicroseconds(point6);
}

void eStop(){ // Stops the motors if overtilted, or resistor trigger pulled out
    trigger = digitalRead(EmergencyGnd);
    if(posy>30 || posy<-30||posz>30||posz<-30||trigger == 1){
      state = 9;
    }
}

void motorSafety(){
    if(point1 < 1140){
    point1 = 1130;
  }
  if(point2 < 1140){
    point2 = 1130;
  }
  if(point3 < 1140){
    point3 = 1130;
  }
  if(point4 < 1140){
    point4 = 1130;
  }
  if(point5 < 1140){
    point5 = 1130;
  }
  if(point6 < 1140){
    point6 = 1130;
  } 
  if(point1 > 2000){
    point1 = 2000;
  }
  if(point2 > 2000){
    point2 = 2000;
  }
  if(point3 > 2000){
    point3 = 2000;
  }
  if(point4 > 2000){
    point4 = 2000;
  }
  if(point5 > 2000){
    point5 = 2000;
  }
  if(point6 > 2000){
    point6 = 2000;
  } 
}

