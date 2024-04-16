#include <SPI.h>
#include <Controllino.h>
#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <SPI.h>
#include <Ethernet.h>

// Network varibles for socket communication
// MAC address of the Controlino Opta
byte mac[] = { 0x70, 0x4C, 0xA5, 0x01, 0x30, 0x01 };



// IP address and port of the server
IPAddress serverIP( 192,168,125,1);  // Replace with the IP address of your server
int serverPort = 55555;  // Replace with the port used by your server

EthernetClient client;

// Pin definition.
const int dirPin = 8; // DIR- Relay 0
const int pulPin = 2; // PUL- Relay 1
const int actPin = 27; // ACT - Relay 2 
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


// Button Setup

// Current and previous state of the button.
int buttonState     = 0;
int lastButtonState = 0;

// Variables to implement button debouncing.
unsigned long lastDebounceTime  = 0;
unsigned long debounceDelay     = 50; // In ms

// bool for switching the actuation
bool actuate;


// Running modes
bool socketCommunicationOn = false;
unsigned long testTime = 10;
float vTcp = 0.06; // Feeding speed in m/s


void pulse()
{
    if (pulseOk){
        nextPulse = currentTime + pulseWidth;
        digitalWrite(pulPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(pulPin, LOW);
        
    }

}

float readSpead(){
    float received_float;
    while (client.available() >= sizeof(received_float)) {
    client.read((byte*)&received_float, sizeof(received_float));
    }
    Serial.print("VTcp: ");
     Serial.println(received_float, 32);
    return received_float;
}

float calculatePulseDelay()
{

    float feedSpeed;
    if( socketCommunicationOn){vTcp = readSpead();}


    pulseOk = vTcp > 0 && vTcp <= 0.5;
    if (pulseOk){
        feedSpeed = vTcp;
        nextSample = currentTime + SAMPLE_TIME_MICROS;
        pulseWidth = (1 / feedSpeed) * PULSE_WIDTH_CONSTANT;
        return pulseWidth;
    }else{
        return (1 / 0.5) * PULSE_WIDTH_CONSTANT;
    }
}

void setup()
{
    // Setup pin modes and logic singals
    pinMode(dirPin, OUTPUT);
    pinMode(pulPin, OUTPUT);
    pinMode(actPin, OUTPUT);
    pinMode(A12, INPUT);

    // Initialize LED_BUILTIN as an output
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(dirPin, LOW);
    digitalWrite(pulPin, LOW);
    digitalWrite(actPin, LOW);

    // Start serial communication
    Serial.begin(9600);

    if( socketCommunicationOn){
    
    //Start the Ethernet connection
    if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // No point in continuing, so do nothing forevermore:
    for (;;)
        ;
    }
    }

    delay(1000); // Allow the Ethernet to initialize
    Serial.println("Connecting to server...");


    if(socketCommunicationOn){
            // Connect to the server
            if (client.connect(serverIP, serverPort)) {
            Serial.println("Connected to server");
            } else {
            Serial.println("Connection to server failed");
            // No point in continuing, so do nothing forevermore:
            for (;;)
                ;
            }
    }

    Serial.println("-------SYSTEM START------");

    // Sample and start test timer
    currentTime = micros();
    pulseWidth = calculatePulseDelay();
    beginTime = micros();
}

void loop()
{
    // // Check button state
    // int reading = digitalRead(A12);

    //   // Check if button state has changed.
    // if (reading != lastButtonState) {
    //   lastDebounceTime = millis();
    // }

    // // Debouncing routine.
    // if ((millis() - lastDebounceTime) > debounceDelay) {
    //   if (reading != buttonState) {
    //     buttonState = reading;

    //     // Only increment the counter if the new button state is HIGH.
    //     if (buttonState == HIGH) {
            

    //       if(actuate){
    //         actuate = false;
    //         digitalWrite(actPin,LOW);
    //         digitalWrite(LED_BUILTIN, LOW);

    //       }
    //       else{
    //         actuate = true;
    //         digitalWrite(actPin,HIGH);
    //         digitalWrite(LED_BUILTIN, HIGH);

    //       }
    //     }
    //   }
    // }
    // // Save the current state as the last state, for next time through the loop.
    // lastButtonState = reading;
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
          Serial.println("The End");
          delay(1000);
        }
    }
}
