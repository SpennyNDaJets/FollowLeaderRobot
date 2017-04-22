void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Beginning");
}

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


void loop() {
  // put your main code here, to run repeatedly:
    Serial.print("Left Sensor: ");
    Serial.println(checkSensor(13));

    Serial.print("Right Sensor: ");
    Serial.println(checkSensor(12));
    Serial.println();
    delay(1000);
}
