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

// line calibration
unsigned long leftMax = 0;
unsigned long rightMax = 0;


//int photoBuffer = 1000;
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

  // turn wheels to start moving forward
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
  pinMode(pin, OUTPUT);           //set to output
  digitalWrite(pin, HIGH);        //set high
  delay(1);                       //charge cap
 
  pinMode(pin, INPUT);            //set low
  unsigned long s = micros();     //start timing
  
  while(digitalRead(pin) == HIGH){ 
  }
  return micros() - s;            // find final time
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
  unsigned long left = checkSensor(leftPhoto);
  unsigned long right = checkSensor(rightPhoto);
  
  Serial.print(left);
  Serial.print("   ");
  Serial.println(right);

  // continue if on line
   if (left < 1200) && right < 2000) {
    analogWrite(PWMright, currSpeed);
    analogWrite(PWMleft, currSpeed); 
    
    //Serial.println("On line");
    //Serial.println(left);
    //Serial.println(right);
  }
  
  // enter intersection
  else if (left > 1200) && right > 2000) {
    // stop
    digitalWrite(right1In, LOW);
    digitalWrite(left1In, LOW);

    // continue if not turn signal
    if (
    digitalWrite(right1In, HIGH);
    digitalWrite(left1In, HIGH);
    
    // check if turn right or turn left

  }
  
  //slow left wheel if left sensor is high
  else if (left > 1200){
    analogWrite(PWMleft, currSpeed - 28);  // 28 = 1 V for 9 V source
    //Serial.println("Left wheel off"); 
    //Serial.println(left); 
  }

  //slow right wheel if right sensor is high
  else if (right > 2000){
    analogWrite(PWMright, currSpeed - 28);
    //Serial.println("Right wheel off"); 
    //Serial.println(right);
  }
}
