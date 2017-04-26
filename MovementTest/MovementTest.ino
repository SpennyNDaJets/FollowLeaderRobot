#include "EventTimer.h"

EventTimer calibrator;

// pin numbers
int PWMright = 5;
int rightIn2 = 4;
int rightIn1 = 2;
int leftIn1 = 7;
int leftIn2 = 8;
int PWMleft = 6;

int leftPhoto = 13;
int rightPhoto = 12;

// line thresholds
unsigned long leftThres = 1000;
unsigned long rightThres = 1500;


// current speed
int currSpeed = 100;


// turn booleans
bool leftSig = false;
bool rightSig = false;

void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  pinMode(PWMright, INPUT);
  pinMode(rightIn2, INPUT);
  pinMode(rightIn1, INPUT);
  pinMode(PWMleft, INPUT);
  pinMode(leftIn2, INPUT);
  pinMode(leftIn1, INPUT);

  // turn wheels to start moving forward
  digitalWrite(rightIn2, LOW);
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn2, LOW);
  digitalWrite(leftIn1, HIGH);
  
  Serial.println("Beginning");
}

// check line sensor
unsigned long checkSensor(int pin){
  pinMode(pin, OUTPUT);           //set to output
  digitalWrite(pin, HIGH);        //set high
  delay(5);                       //charge cap
 
  pinMode(pin, INPUT);            //set low
  unsigned long s = micros();     //start timing
  
  while(digitalRead(pin) == HIGH){ 
  }
  return micros() - s;            // find final time
} 

/*
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
*/


void loop() {
  // check if on line
  unsigned long left = checkSensor(leftPhoto);
  unsigned long right = checkSensor(rightPhoto);

  // continue if on line
   if ((left < leftThres) && (right < rightThres)) {
    analogWrite(PWMright, currSpeed);
    analogWrite(PWMleft, currSpeed); 
  }
  
  // enter intersection
  else if ((left >= leftThres) && (right >= rightThres)) {
    // stop
    digitalWrite(rightIn1, LOW);
    digitalWrite(leftIn1, LOW);
    
    delay(3000);  //test
    
    // continue if not turn signal
    if (!leftSig && !rightSig) {
      digitalWrite(rightIn1, HIGH);
      digitalWrite(leftIn1, HIGH);
    }
    // check if turn right or turn left

  }
  
  //slow left wheel if left sensor is high
  else if (left >= leftThres){
    analogWrite(PWMleft, currSpeed - 28);  // 28 = 1 V for 9 V source
  }

  //slow right wheel if right sensor is high
  else if (right >= rightThres){
    analogWrite(PWMright, currSpeed - 28);
  }
}
