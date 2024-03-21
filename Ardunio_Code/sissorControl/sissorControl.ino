#include <Arduino.h>

const int dirPin = 8; // DIR- connected to pin 8
const int pulPin = 9; // PUL- connected to pin 9

const float NUMBER_TEETH_A = 32;
const float NUMBER_TEETH_B = 60;
const float FEEDING_RADIUS = (30.0 - 1.427) *1e-3; // In meters
const float STEPS_PER_REVOLUTION = 800;
const float PART_A = FEEDING_RADIUS*NUMBER_TEETH_A*2*PI;
const float PART_B = STEPS_PER_REVOLUTION*NUMBER_TEETH_B;
unsigned long PULSE_WIDTH_CONSTANT =(unsigned long)1e6* PART_A/PART_B;
const float vTcp = 0.25;

const unsigned long SAMPLE_TIME_MICROS = 50000; // 50 milliseconds in microseconds

float pulseWidth;

unsigned long lastPulse;
unsigned long lastSample;
unsigned long nextSample;
unsigned long nextPulse;
unsigned long beginTime;
unsigned long nbrPulses = 0;
unsigned long nbrSamples = 0;
unsigned long testTime = 5;
unsigned long nbrSteps = 0;
unsigned long stepsToDo = STEPS_PER_REVOLUTION/4 * 3;


void pulse()
{
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pulPin, LOW);
    nbrSteps++;
}

unsigned long calculatePulseDelay()
{
    return (1/vTcp)*PULSE_WIDTH_CONSTANT;
}

void setup()
{
    
    pinMode(dirPin, OUTPUT);
    pinMode(pulPin, OUTPUT);
    digitalWrite(dirPin, HIGH);
    digitalWrite(pulPin, LOW);
    Serial.begin(9600); // Start serial communication for debugging
    Serial.print("Pulse width constant: ");
    Serial.println(PULSE_WIDTH_CONSTANT);
    Serial.println("-------SYSTEM START------");
    pulseWidth = calculatePulseDelay();
    lastSample = micros();
    lastPulse = micros();
    Serial.print("Pulse width:");
    Serial.println(pulseWidth);
    beginTime = micros();
    

}

void loop()
{
    unsigned long currentTime = micros();
    bool timeToSample = (currentTime >= nextSample);
    bool timeToPulse = (currentTime >= nextPulse);

    if (timeToSample && !timeToPulse) {
        nextSample = currentTime + SAMPLE_TIME_MICROS;
        pulseWidth = calculatePulseDelay();
        
        nbrSamples++;
    } else if (!timeToSample && timeToPulse) {
        nextPulse = currentTime + pulseWidth;
        pulse();
        nbrPulses ++;
    } else if (timeToSample && timeToPulse) {
        nextSample = currentTime + SAMPLE_TIME_MICROS;
        pulseWidth = calculatePulseDelay();
        nextPulse = currentTime + pulseWidth;
        pulse();
        
        lastSample = micros();
       nbrPulses ++;
       nbrSamples++;
    }
    if (nbrSteps == stepsToDo){
        digitalWrite(dirPin, LOW);

    }
        if (nbrSteps == stepsToDo*2){
        
        digitalWrite(dirPin, HIGH);
        nbrSteps = 0;

    }
    if(currentTime- beginTime > testTime*1e6 || nbrSteps >= stepsToDo*2){
        float expectedPulses = testTime*1e6/pulseWidth;
        float expectedSamples = testTime*1e6/SAMPLE_TIME_MICROS;
        Serial.print("PULSE, Expected: ");
        Serial.print(expectedPulses);
        Serial.print(", Actual: ");
        Serial.println(nbrPulses);
        Serial.print("Sample, Expected: ");
        Serial.print(expectedSamples);
        Serial.print(", Actual: ");
        Serial.println(nbrSamples);
        while(true){

        }
    }
}
