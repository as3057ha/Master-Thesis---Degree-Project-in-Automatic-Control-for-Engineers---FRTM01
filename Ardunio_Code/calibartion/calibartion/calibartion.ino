#include <Controllino.h>
#include <Arduino.h>

// Pin definition.
const int dirPin = 8; // DIR- Relay 0
const int pulPin = 2; // PUL- Relay 1

// Motor constants
const float STEPS_PER_REVOLUTION = 800;

// Drive system constants
const float NUMBER_TEETH_A = 16;
const float NUMBER_TEETH_B = 48;

const long revolutions = 8;
const long delayPulse = 1000;

// Debug and testing varibles
unsigned long nbrPulses = 0;


void pulse()
{
  digitalWrite(pulPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pulPin, LOW);
}

void setup()
{
    // Setup pin modes and logic singals
    pinMode(dirPin, OUTPUT);
    pinMode(pulPin, OUTPUT);
    digitalWrite(dirPin, LOW);
    digitalWrite(pulPin, LOW);

    // Start serial communication
    Serial.begin(9600);
    Serial.println("-------SYSTEM START------");

}

void loop()
{
    if (nbrPulses < revolutions*(STEPS_PER_REVOLUTION * NUMBER_TEETH_B) / NUMBER_TEETH_A) {
        pulse();
        delayMicroseconds(delayPulse);
        nbrPulses++;
    }

    // If: test is over
    if (nbrPulses >= revolutions*(STEPS_PER_REVOLUTION * NUMBER_TEETH_B) / NUMBER_TEETH_A )
    {

        Serial.println("-------SYSTEM END------");

        // Calculate test result
        Serial.print(", Actual: ");
        Serial.println(nbrPulses);
        Serial.print("Sample, Expected: ");


        // Infinite loop
        while (true)
        {
          Serial.println("The End");
          delay(1000);
        }
    }
}


