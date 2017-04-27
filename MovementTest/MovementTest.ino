#include "EventTimer.h"

EventTimer calibrator;

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
unsigned long leftThres = 1000;
unsigned long rightThres = 1500;


// current speed
int currSpeed = 50;

// turn speed
int turnSpeed = 10;

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

  // turn wheels to start moving forward
  digitalWrite(rightIn2, LOW);
  digitalWrite(rightIn1, HIGH);
  digitalWrite(leftIn2, LOW);
  digitalWrite(leftIn1, HIGH);

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

  //calibrate sensors
  calibrate();

  Serial.println("Beginning");
}


// check line sensor
unsigned long checkSensor(int pin) {
  pinMode(pin, OUTPUT);           //set to output
  digitalWrite(pin, HIGH);        //set high
  delay(1);          //charge cap

  pinMode(pin, INPUT);             //set low
  unsigned long s = micros();     //start timing

  while (digitalRead(pin) == HIGH) {

  }

  return micros() - s;            // find final time
}


//calibrate line sensors
void calibrate() {
  EventTimer t;
  t.start(5000);

  int leftMax = 0;
  int leftMin = 0xffff;
  int rightMax = 0;
  int rightMin = 0xffff;

  while (!t.checkExpired()) {
    int tempLeft = checkSensor(leftPhoto);
    int tempRight = checkSensor(rightPhoto);

    if (tempLeft < leftMin)
      leftMin = tempLeft;
    else if (tempLeft > leftMax)
      leftMax = tempLeft;
    if (tempRight < rightMin)
      rightMin = tempRight;
    else if (tempRight > rightMax)
      rightMax = tempRight;
  }

  leftThres = (leftMax + leftMin) / 2;
  rightThres = (rightMax + rightMin) / 2;

  Serial.print("leftThres: ");
  Serial.println(leftThres);
  Serial.print("rightThres: ");
  Serial.println(rightThres);
}


volatile uint8_t dataReady = 0;
volatile uint16_t code = 0;
volatile uint8_t index = 0;

// check for turn signal
int getTurnSignal() {
  if (dataReady)
  {
    dataReady = 0;
    return code;
  }
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
  }
  // right turn signal
  else if (ts == 146) {
    prepRight = true;
    prepLeft = false;
  }
}

//turn right
void turnRight() {
  Serial.println("Turn right");
  Serial.println(turnSpeed);
  digitalWrite(PWMright, turnSpeed/4);
  digitalWrite(PWMleft, turnSpeed);

  // reset turnSignal
  prepRight = false;
}

// turn left
void turnLeft() {
  Serial.println("Turn left");
  digitalWrite(PWMright, turnSpeed);
  digitalWrite(PWMleft, turnSpeed/4);
  delay(1000);

  // reset turnSignal
  prepLeft = false;
}

void loop() {
  //check for line sensor
  int turnSignal = getTurnSignal();
  checkTurnSignal(turnSignal);


  // check if on line
  unsigned long left = checkSensor(leftPhoto);
  unsigned long right = checkSensor(rightPhoto);


  Serial.print(left);
  Serial.print(" ");
  Serial.println(right);
  // continue if on line
  if ((left < leftThres) && (right < rightThres)) {
    analogWrite(PWMright, currSpeed);
    analogWrite(PWMleft, currSpeed);
  }

  // enter intersection
  else if ((left >= leftThres) && (right >= rightThres)) {
    Serial.println("INTERSECTION!!");

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
    analogWrite(PWMleft, currSpeed - 20);  // 28 = 1 V for 9 V source
    Serial.println("Left is dark");
  }

  //slow right wheel if right sensor is high
  else if (right >= rightThres) {
    analogWrite(PWMright, currSpeed - 20);
    Serial.println("Right is dark");
  }


}
