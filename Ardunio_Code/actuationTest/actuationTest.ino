
#include <Controllino.h>
#include <Arduino.h>


// Pin definition.
// 0 and 1

const int actPin = 27; // ACT - Relay 2 
const int downState = 85 ; // Cutting pin
const int upState = 84 ; // Cutting pin







void setup()
{
    // Setup pin modes and logic singals
    pinMode(actPin, OUTPUT);
    pinMode(downState, INPUT);
    pinMode(upState, INPUT);

    digitalWrite(actPin, LOW);

    // Start serial communication
    Serial.begin(9600);



    Serial.println("-------SYSTEM START------");


}

void loop()
{

  if(digitalRead(downState)){
    Serial.println("DOWN");
  }
   
  if(digitalRead(upState)){
    Serial.println("UP");
  }
}
