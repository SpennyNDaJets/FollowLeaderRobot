#include "EventTimer.h"

// timer for beigining turns
EventTimer turnTimer;
// timer for turn signal
EventTimer signalTimer;


// pin numbers
int PWMright = 5;
int rightIn2 = 4;
int rightIn1 = 2;
int leftIn1 = 7;
int leftIn2 = 9;
int PWMleft = 6;

//left turn signal on A2
//right turn signal on A5

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
double Ki = 0.6; //integral constant
double Kd= 5; //derivative constant
double Kp= 500; //proportional constant
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
  //send out signal
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  // start timing
  startTime = micros();

  // do not take new reading which signal has not returned
  newReading = false;
}



// retreive echo
void detectEcho(){
  // take time at which signal returned
  endTime = micros();
  // find difference in time
  delta = endTime - startTime;
  // set boolean to allow for new reading to be taken
  newReading = true;
}



// check line sensor
unsigned long checkSensor(int pin) {
  pinMode(pin, OUTPUT);           //set to output
  digitalWrite(pin, HIGH);        //set high
  delayMicroseconds(10);          //charge cap

  pinMode(pin, INPUT);             //set low
  unsigned long s = micros();     //start timing

  // wait till pin goes low
  while (digitalRead(pin) == HIGH) {

  }

  // return time that it takes capacitor to discharge
  return micros() - s;
}



// distance calculation
double distance() { //in meters
  double s = 343; // speed of sound in meters/s 
  double durationSeconds = ((delta/2.0)/1000000);
  double distance = (durationSeconds*s);
  // 0.15 meter offset from timer start and echo going high
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

  // update error build up and reassign to limit integral windup
  sumError += error;
  if(sumError > 10) 
    sumError = 5;
  if(sumError < 0) 
    sumError = 0; 

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
  }

  // maximum speed
  if(currSpeed > 100) currSpeed = 100; 

  //start taking new reading
  takeReading();
}

// calibrate Line sensors
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
    //save turn signal until next intersection
    prepLeft = true;
    prepRight = false;

    //turn left turn signal on and right turn signal off
    digitalWrite(A2, HIGH);
    digitalWrite(A5, LOW);
    //start timer to start blinking
    signalTimer.start(500);

    //stop for 1 second to avoid hitting lead bot
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

    //turn right turn signal on
    digitalWrite(A5, HIGH);
    digitalWrite(A2, LOW);
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
  // time necessary to enter center of interection
  turnTimer.start(250);
  while(!turnTimer.checkExpired()){
  }

   //stop
  digitalWrite(rightIn1, LOW);
  digitalWrite(leftIn1, LOW);
  
  // turn right
  digitalWrite(rightIn2, HIGH);
  digitalWrite(leftIn1, HIGH);

// turn for at least 0.2 s to ensure right sensor is not on black line
  turnTimer.start(200);
  while(!turnTimer.checkExpired()){
  }

  
  // while not on line
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
  digitalWrite(A5, LOW);
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
  digitalWrite(A2, LOW);
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
    //maintain average speed by increasing one side as much as other side is decreased
    analogWrite(PWMleft, currSpeed / 2);
    analogWrite(PWMright, currSpeed * 3/2);
  }

  //slow right wheel if right sensor is high
  else if (right >= rightThres) {
    //maintain average speed by increasing one side as much as other side is decreased
    analogWrite(PWMright, currSpeed / 2);
    analogWrite(PWMleft, currSpeed * 3/2);
  }
}
