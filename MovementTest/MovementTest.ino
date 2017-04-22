#include "EventTimer.h"

EventTimer calibrator;

int PWMright = 5;
int rightIn2 = 4;
int rightIn1 = 2;
int leftIn1 = 7;
int leftIn2 = 8;
int PWMleft = 6;

int leftPhoto = 13;
int rightPhoto = 12;

// line calibration
unsigned long leftMax = 0;
unsigned long rightMax = 0;

int photoBuffer = 1000;
// current speed
int currSpeed = 100;

void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  pinMode(PWMright, INPUT);
  pinMode(rightIn2, INPUT);
  pinMode(rightIn1, INPUT);
  pinMode(PWMleft, INPUT);
  pinMode(leftIn2, INPUT);
  pinMode(leftIn1, INPUT);
  
  digitalWrite(rightIn2, LOW);
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn2, LOW);
  digitalWrite(leftIn1, HIGH);


  Serial.println("Calibrating");
  
  //calibrate line sensor
  leftMax = calibrate(leftPhoto);
  rightMax = calibrate(rightPhoto);
  Serial.print("Left Max: ");
  Serial.println(leftMax);
  Serial.print("Right Max: ");
  Serial.println(rightMax);
  
  Serial.println("Beginning");
}

// check line sensor
unsigned long checkSensor(int pin){
    // left photosensor
  pinMode(pin, OUTPUT);     //set to output
  digitalWrite(pin, HIGH);  //set high
  delay(1);                       //charge cap
 
  pinMode(pin, INPUT);      //set low
  unsigned long s = micros();     //start timing
  
  while(digitalRead(pin) == HIGH){ 
  }
  return micros() - s; // find final time
}


// calibrate line sensor
unsigned long calibrate(int pin){
  unsigned long maxRead = 0;
  // start timer for 4 sec
  calibrator.start(4000);
  // while timer is not expired
  while (!calibrator.checkExpired()){
    // set value equal to Sensor
    int senVal = checkSensor(pin);
    // check if value is greater than max
    if (maxRead < senVal){
      maxRead = senVal;
    }
    
  }
  // return max
  return maxRead;
}


void loop() {
  // check if on line
  int left = checkSensor(leftPhoto);
  int right = checkSensor(rightPhoto);
  Serial.print(left);
  Serial.print("   ");
  Serial.println(right);
  if (left <  (2000) && right < (2000)) {
    // continue if on line
    analogWrite(PWMright, currSpeed);
    analogWrite(PWMleft, currSpeed); 
    //Serial.println("On line");
    //Serial.println(left);
    //Serial.println(right);
  }
  if (left > (1200)){
    //slow left wheel if not
    analogWrite(PWMleft, currSpeed - 28);  // 28 = 1 V for 9 V source
    //Serial.println("Left wheel off"); 
    //Serial.println(left); 
  }
  if (right > (2000)){
    //slow right wheel if not
    analogWrite(PWMright, currSpeed - 38);
    //Serial.println("Right wheel off"); 
    //Serial.println(right);
  }
}
