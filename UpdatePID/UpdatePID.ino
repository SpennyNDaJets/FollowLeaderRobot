#include "EventTimer.h"

EventTimer turnTimer;
EventTimer signalTimer;


// pin numbers
int PWMright = 5;
int rightIn2 = 4;
int rightIn1 = 2;
int leftIn1 = 7;
int leftIn2 = 9;
int PWMleft = 6;

int leftPhoto = 13;
int rightPhoto = 12;

// line thresholds
unsigned long leftThres;
unsigned long rightThres;

// ultrasonic sensor variables
int TRIGGER = 10;
int ECHO = 3;
double dist;
double duration;
double targetDistance = 0.5; // in meters
double Ki = 0.75; //change these values
double Kd= 5; //change these values
double Kp= 300; //change these values
double sumError = 0;
double prevError = -100;
double prevDist = 0;

unsigned long startTime;
unsigned long endTime;
int delta;
bool newReading;

// current speed
int currSpeed; //set the value in setup (when we can see his robot)

// turn speed
int turnSpeed = 50;
// cruise speed
int cruiseSpeed = 40;

// turn booleans
bool prepLeft = false;
bool prepRight = false;

void setup() {
  Serial.begin(9600);

  // put your setup code here, to run once:
  pinMode(PWMright, INPUT);
  pinMode(rightIn2, INPUT);
  pinMode(rightIn1, INPUT);
  pinMode(PWMleft, INPUT);
  pinMode(leftIn2, INPUT);
  pinMode(leftIn1, INPUT);
  pinMode(A2, OUTPUT);
  pinMode(A5, OUTPUT);

  // turn wheels to start moving forward
  digitalWrite(rightIn2, LOW);
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn2, LOW);
  digitalWrite(leftIn1, HIGH);

  // ultrasonic sensor setup
  pinMode(TRIGGER, OUTPUT); //trigger 
  pinMode(ECHO, INPUT); //echo 

  //disable interrupts
  cli();

  //set timer1 to Normal mode, pre-scaler of 8
  //use ASSIGNMENT since the bootloader sets up timer1 for 8-bit PWM, which isn't very useful here
  TCCR1A = 0x00;
  TCCR1B = 0x02;

  //enable input capture interrupt
  TIMSK1 |= (1 << ICIE1);

  //enable noise cancelling
  TCCR1B |= (1 << ICNC1);

  //set for falling edge on ICP1 (pin 8 on an Arduino Uno)
  TCCR1B &= ~(1 << ICES1);

  //re-enable interrupts
  sei();

  // set up interupt to catch retrievial of Ultasonic
  attachInterrupt(1, detectEcho, FALLING);

  //calibrate sensors
  calibrate();

  //start taking reading
  takeReading();

  Serial.println("Beginning");
}

// send of signal of ultrasonic
void takeReading(){
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  // start timing
  startTime = micros();

  newReading = false;
}

// retreive echo
void detectEcho(){
  endTime = micros();
  delta = endTime - startTime;
//  Serial.print("Start Time: ");
//  Serial.println(startTime);
//  Serial.print("End Time: ");
//  Serial.println(endTime);
//  Serial.print("Delta: ");
//  Serial.println(delta);
  newReading = true;
}

// check line sensor
unsigned long checkSensor(int pin) {
  pinMode(pin, OUTPUT);           //set to output
  digitalWrite(pin, HIGH);        //set high
  delayMicroseconds(10);          //charge cap

  pinMode(pin, INPUT);             //set low
  unsigned long s = micros();     //start timing

  while (digitalRead(pin) == HIGH) {

  }

  return micros() - s;            // find final time
}

// distance calculation
double distance() { //in meters
  double s = 343; // speed of sound in meters/s 
  double durationSeconds = ((delta/2.0)/1000000);
  double distance = (durationSeconds*s);
  return distance - 0.15;
}

// PID algorithm
void PID() {
  // calculate distance away
  double actualDistance = distance();

  // check to make sure new distance is within buffer
  if ((actualDistance < 0.05) && prevDist != 0){
    //else set equal to previous distance
    actualDistance = prevDist;
  }

  // calculate error
  double error = actualDistance - targetDistance; //gives us voltage (val between 0 to 255)

  // update error build up and reassign if necessary
  sumError += error;
  if(sumError > 10) 
    sumError = 5;
  if(sumError < 0) 
    sumError = 0; 

  //currSpeed = (int)(30 + Kp * error);

  // do not use derivative control on first iteration
  if (prevError == -100) {
    currSpeed = (int)(Kp * error + Ki * sumError);
  }
  else {
    currSpeed = (int)(Kp * error + Ki * sumError + Kd * (error - prevError));
  }

  //update previous variables
  prevError = error;
  prevDist = actualDistance;

  // there are notnegative speeds
  if(currSpeed < 0) {
    currSpeed = 0;
//    Serial.println("Speed is Negative");
  }

  // maximum speed
  if(currSpeed > 100) currSpeed = 100; 

  //start taking new reading
  takeReading();
}


void calibrate() {
  // calibrate for 5 seconds
  EventTimer t;
  t.start(5000);

  //initialize min and max 
  unsigned long leftMax = 0;
  unsigned long rightMax = 0;

  //constantly check sensors
  while (!t.checkExpired()) {
    int tempLeft = checkSensor(leftPhoto);
    int tempRight = checkSensor(rightPhoto);

    //update min and max
    if (tempLeft > leftMax)
      leftMax = tempLeft;
    if (tempRight > rightMax)
      rightMax = tempRight;
  }

  // calculate threshold 
  leftThres = leftMax / 2;
  rightThres = rightMax / 2;

//  Serial.print("leftThres: ");
//  Serial.println(leftThres);
//  Serial.print("rightThres: ");
//  Serial.println(rightThres);
}


volatile uint8_t dataReady = 0;
volatile uint16_t code = 0;
volatile uint8_t index = 0;

// check for turn signal
int getTurnSignal() {
  // if whole signal received 
  if (dataReady)
  {
    // reset data ready
    dataReady = 0;
    // return code
    return code;
  }
  //return 0 as no signal received
  return 0;
}






volatile uint16_t fallingEdge = 0;
volatile uint16_t risingEdge = 0;

//ICR1 is the input capture ("timestamp") register
//capture happens on PB0/Arduino pin 8
ISR(TIMER1_CAPT_vect)
{
  if (!(TCCR1B & (1 << ICES1))) //if we're looking for FALLING edges
  {
    fallingEdge = ICR1;
    TCCR1B |= (1 << ICES1); //now set to look for a rising edge
  }

  else //we must be looking for a RISING edge
  {
    risingEdge = ICR1;
    TCCR1B &= ~(1 << ICES1); //set to look for a falling edge

    //and process
    uint16_t delta = risingEdge - fallingEdge; //length of pulse, in timer counts
    delta /= 2; //scaled to us

    if (delta > 2250 && delta < 2750) //start pulse
    {
      index = 0;
      code = 0; //reset code
      dataReady = 0; //clobber previous read if it wasn't processed
    }

    else if (delta > 500 && delta < 800) //short pulse
    {
      index++;
    }

    else if (delta > 1100 && delta < 1500) //long pulse
    {
      code += (1 << index);
      index++;
    }

    else //error
    {
      index = 0; //start over
      code = 0;
    }

    if (index == 12) dataReady = 1;
  }
}





// check if left or right turn signal
void checkTurnSignal(int ts) {
  // left turn signal
  if (ts == 147) {
    prepLeft = true;
    prepRight = false;
    currSpeed = 20;

    //turn left turn signal on
    digitalWrite(A2, HIGH);
    signalTimer.start(500);

    //stop for 1 second
    digitalWrite(rightIn1, LOW);
    digitalWrite(leftIn1, LOW);
    delay(1000);
    //continue straight
    digitalWrite(rightIn1, HIGH);
    digitalWrite(leftIn1, HIGH);
  }
  // right turn signal
  else if (ts == 146) {
    prepRight = true;
    prepLeft = false;
    currSpeed = 20;

    //turn right turn signal on
    digitalWrite(A5, HIGH);
    signalTimer.start(500);

    //stop for 1 second
    digitalWrite(rightIn1, LOW);
    digitalWrite(leftIn1, LOW);
    delay(1000);
    //continue straight
    digitalWrite(rightIn1, HIGH);
    digitalWrite(leftIn1, HIGH);
  }
}






//turn right
void turnRight() {
  //stop
  digitalWrite(rightIn1, LOW);
  digitalWrite(leftIn1, LOW);


  // continue forward to center of intersection
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn1, HIGH);
  analogWrite(PWMright, turnSpeed);
  analogWrite(PWMleft, turnSpeed);
  turnTimer.start(250);
  while(!turnTimer.checkExpired()){
  }

   //stop
  digitalWrite(rightIn1, LOW);
  digitalWrite(leftIn1, LOW);
  
  // turn right
  digitalWrite(rightIn2, HIGH);
  digitalWrite(leftIn1, HIGH);

  turnTimer.start(200);
  while(!turnTimer.checkExpired()){
  }

  
  //while not on line
  while (checkSensor(rightPhoto) < rightThres) {
  }

  //while on intersecting line
  while (checkSensor(rightPhoto) > rightThres){
  }

  // stop
  digitalWrite(rightIn2, LOW);
  digitalWrite(leftIn1, LOW);

  //continue straight
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn1, HIGH);
  
  // reset turnSignal
  prepRight = false;
  // turn off turnSignal
  digitalWrite(A2, LOW);
}





// turn left
void turnLeft() {
  //stop
  digitalWrite(rightIn1, LOW);
  digitalWrite(leftIn1, LOW);

  // continue to center of intersection
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn1, HIGH);
  analogWrite(PWMright, turnSpeed);
  analogWrite(PWMleft, turnSpeed);
  turnTimer.start(250);
  while(!turnTimer.checkExpired()){
  }

   //stop
  digitalWrite(rightIn1, LOW);
  digitalWrite(leftIn1, LOW);

  // turn left 
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn2, HIGH);

  turnTimer.start(200);
  while(!turnTimer.checkExpired()){
  }

  //while not on line
  while (checkSensor(leftPhoto) < leftThres) {
  }

  //while on intersecting line
  while (checkSensor(leftPhoto) > leftThres){
  }
  
  // stop
  digitalWrite(rightIn1, LOW);
  digitalWrite(leftIn2, LOW);

  //continue straight
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn1, HIGH);
  
  // reset turnSignal
  prepLeft = false;
  // turn off turnSignal
  digitalWrite(A5, LOW);
}



//flash turn signal
void flashSignal(){
  if (prepRight){
    if (signalTimer.checkExpired()){
      if (digitalRead(A5) == LOW){
        digitalWrite(A5, HIGH);
      }
      else{
        digitalWrite(A5, LOW);
      }
      signalTimer.start(500);
    }
  }
  else{
    if (signalTimer.checkExpired()){
      if (digitalRead(A2) == LOW){
        digitalWrite(A2, HIGH);
      }
      else{
        digitalWrite(A2, LOW);
      }
      signalTimer.start(500);
    }
  }
  
}


void loop() {
  //check for line sensor
  int turnSignal = getTurnSignal();
  checkTurnSignal(turnSignal);

  // if new reading from sensor do PID
  if (newReading && !prepRight && !prepLeft){
    PID();
  }
  // if receive turn signal, set cruise speed (No PID)
  else if(prepRight || prepLeft){
    currSpeed = cruiseSpeed;
    //flash signal
    flashSignal();
  }
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

    // check if turn right
    if (prepRight) {
      turnRight();
    }
    // check if turn left
    else if (prepLeft) {
      turnLeft();
    }
  }

  //slow left wheel if left sensor is high
  else if (left >= leftThres) {
    analogWrite(PWMleft, currSpeed / 2);  // 28 = 1 V for 9 V source
    analogWrite(PWMright, currSpeed * 3/2);
  }

  //slow right wheel if right sensor is high
  else if (right >= rightThres) {
    analogWrite(PWMright, currSpeed / 2);
    analogWrite(PWMleft, currSpeed * 3/2);
  }
}
