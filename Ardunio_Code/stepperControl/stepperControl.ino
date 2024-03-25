#include <Arduino.h>
#include <math.h>
#include <stdint.h>

// Pin definition.
const int dirPin = 8; // DIR- connected to pin 8
const int pulPin = 9; // PUL- connected to pin 9
const int RX = 0;
const int TX = 1;
const int speedBit1 = 2;
const int speedBit2 = 3;
const int speedBit3 = 4;
const int speedBit4 = 5;



// Motor constants
const float STEPS_PER_REVOLUTION = 800;

// Drive system constants
const float NUMBER_TEETH_A = 32;
const float NUMBER_TEETH_B = 60;
const float FEEDING_RADIUS = (30.0 - 1.427) * 1e-3; // In meters

// Constant decleration.
const float PART_A = FEEDING_RADIUS * NUMBER_TEETH_A * 2 * PI;
const float PART_B = STEPS_PER_REVOLUTION * NUMBER_TEETH_B;
const float PULSE_WIDTH_CONSTANT = 1e6 * PART_A / PART_B;
const float shortestPulseDelay = (1 / 0.5) * PULSE_WIDTH_CONSTANT;

// Sample time in micro seconds.
const unsigned long SAMPLE_TIME_MICROS = 50 * 1e3;

// Pulse delay
float pulseWidth;
float previousPulseWidth;

// Timing varibles
unsigned long nextSample;
unsigned long nextPulse;
unsigned long beginTime;
unsigned long currentTime;

// Other varibles
boolean pulseOk = true;

// Debug and testing varibles
unsigned long nbrPulses = 0;
unsigned long nbrSamples = 0;
unsigned long testTime = 10;
float vTcp = 0; // Feeding speed in m/s

// 
bool speedBits[12];

void pulse()
{
    
    if (pulseOk){
        nextPulse = currentTime + pulseWidth;
        digitalWrite(pulPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(pulPin, LOW);
    }

}

float readSpeed(){

    // Read and save the bits in a vector.
   readFourBits(speedBits,0);
   readFourBits(speedBits,5);
   readFourBits(speedBits,9);

   unsigned int fixedSpeed = bitsToInt();
   float speed = bitsToFloat(fixedSpeed);
    return speed;
}

// Function to convert integer to float (fixed-point)
float bitsToFloat(unsigned int intValue) {
    // Fixed-point format: 8 integer bits, 4 fractional bits
    // Convert integer part
    int integerPart = intValue >> 4; // Shift right by 4 bits to get integer part
    // Convert fractional part
    float fractionalPart = ((intValue & 0xF) / 16.0f); // Mask with 0xF (15 in decimal) to get fractional part
    // Combine integer and fractional parts
    return integerPart + fractionalPart;
}



void readFourBits(bool* speedBits, int startPos) {

    // Request speedInfo
    digitalWrite(TX,HIGH);

     while (digitalRead(RX) == LOW) {
    // Do nothing while waiting
    }
    digitalWrite(TX,LOW);

    // Read the speed bits and store them in the array
    speedBits[startPos] = digitalRead(speedBit1);
    speedBits[startPos+1] = digitalRead(speedBit2);
    speedBits[startPos+2] = digitalRead(speedBit3);
    speedBits[startPos+3] = digitalRead(speedBit4);
}



// Function to convert boolean vector to an integer
unsigned int bitsToInt() {
    unsigned int result = 0;
    for (int i = 0; i < 12; i++) {
        result |= (speedBits[11 - i] ? 1 : 0) << i;
    }
    return result;
}


float calculatePulseDelay()
{

    float feedSpeed;
    // Change method for getting vTcp here
    vTcp = readSpeed();
    pulseOk = vTcp > 0 && vTcp <= 0.5;
    if (pulseOk){
        feedSpeed = vTcp;
        nextSample = currentTime + SAMPLE_TIME_MICROS;
        previousPulseWidth = pulseWidth;
        pulseWidth = (1 / feedSpeed) * PULSE_WIDTH_CONSTANT;
        if(previousPulseWidth > pulseWidth){
            nextPulse = nextPulse -previousPulseWidth +pulseWidth;
            }
        return pulseWidth;
    }else{
        return -1;
    }
}

void setup()
{
    // Setup pin modes and logic singals
    pinMode(dirPin, OUTPUT);
    pinMode(pulPin, OUTPUT);
    pinMode(RX,INPUT);
    pinMode(TX,OUTPUT);
    pinMode(speedBit1,INPUT);
    pinMode(speedBit2,INPUT);
    pinMode(speedBit3,INPUT);
    pinMode(speedBit4,INPUT);

    digitalWrite(dirPin, HIGH);
    digitalWrite(pulPin, LOW);

    // Start serial communication
    Serial.begin(9600);
    Serial.println("-------SYSTEM START------");

    // Sample and start test timer
    currentTime = micros();
    pulseWidth = calculatePulseDelay();
    beginTime = micros();
}

void loop()
{
    // Read current time
    currentTime = micros();
    // If: time to sample.
    if (currentTime >= nextSample)
    {
        pulseWidth = calculatePulseDelay();
        nbrSamples++;
    }

    // If: time to pulse.
    if (currentTime >= nextPulse)
    {
        pulse();
        nbrPulses++;
    }

    // If: test is over
    if (currentTime - beginTime > testTime * 1e6)
    {

        Serial.println("-------SYSTEM END------");
        // Calculate test results
        float expectedPulses = testTime * 1e6 / pulseWidth;
        float expectedSamples = testTime * 1e6 / SAMPLE_TIME_MICROS;

        // Test evaluation prints
        Serial.print("PULSE, Expected: ");
        Serial.print(expectedPulses);
        Serial.print(", Actual: ");
        Serial.println(nbrPulses);
        Serial.print("Sample, Expected: ");
        Serial.print(expectedSamples);
        Serial.print(", Actual: ");
        Serial.println(nbrSamples);

        // Infinite loop
        while (true)
        {
        }
    }
}
