#include "EventTimer.h"

EventTimer t;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Beginning");
}

void loop() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1);

  pinMode(13, INPUT);
  unsigned long s = micros();
  
  while(digitalRead(13) == HIGH){
  }

  unsigned long dis = micros() - s;

  Serial.println(dis);

  delay(1000);
}
