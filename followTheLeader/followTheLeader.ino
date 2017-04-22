#include "EventTimer.h"

//create states
bool prepLeft = false;
bool prepRight = false;
bool turnLeft = false;
bool turnRight = false;

//create pin number constants
int rightPWM = 5;
int leftPWM = 6;
int leftPhoto = 13;
int rightPhoto = 2;

//create timers
EventTimer photoTimer;

void setup() {
  Serial.begin(9600);
  //set up pins
  //start necessary timers
  Serial.println("Beginning");
}

//turn method
void turn(){
  
}

void loop() {
  // drive straight
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  delay(1);
 
  pinMode(5, INPUT);
  unsigned long s = micros();
  
  while(digitalRead(5) == HIGH){
    
  }

  unsigned long dis = micros() - s;

   //PID Control speed with ultasonic
   //PID Control direction with photodetector (both should be light??)
     //BOTH MAY GO DARK IF ENTERING INTERSECTION
   //check for turn signals with IR
    
  // while preparing to turn left
    //maintain speed
    //PID Control direction with photodetector
    //check both photosensors for intersection 

  //while preparing to turn right
    //maintain speed
    //PID Control direction with photodetector
    //check right photosensor for intersection

  //while turning left
    //change motor speeds accordingly
    // left sensor sees dark and then light

  //while turning right
    //change motor speeds accordingly
    //right sensor sees dark and then light
    
}
