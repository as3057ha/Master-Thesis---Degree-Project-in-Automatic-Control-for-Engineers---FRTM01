#include <Controllino.h>
#include <Arduino.h>

// Pin definitions
const int dirPin = 8; // Direction Pin - Relay 0
const int pulPin = 2; // Pulse Pin - Relay 1
const byte encoderPinA = CONTROLLINO_IN1; // Optical encoder pin A
const byte encoderPinB =CONTROLLINO_IN0; // Optical encoder pin B

// Motor constants
const float STEPS_PER_REVOLUTION = 800;

// Drive system constants
const float NUMBER_TEETH_A = 16;
const float NUMBER_TEETH_B = 48;

// Desired number of revolutions
const float revolutions = 10;

// Delay between pulses (microseconds)
const long delayPulse = 1000;

// Encoder constants
const float WHEEL_RADIUS = 0.05; // Radius of rotation in meters

// Debug and testing variables
unsigned long maxNbrPulses; // Maximum number of pulses to be generated
long RPM = 1; // Revolutions per minute
float pulseDelay; // Delay between each pulse (microseconds)
unsigned long previousMicros = 0; // Time of the last pulse
long nbrPulses = 0; // Current number of pulses generated
volatile long encoderPos = 0; // Current position of the encoder

// Function to generate a pulse
void pulse() {
  digitalWrite(pulPin, HIGH); // Set pulse pin high
  delayMicroseconds(5); // Wait for a short duration
  digitalWrite(pulPin, LOW); // Set pulse pin low
}

// Interrupt service routine for encoder
void updateEncoder() {
  int encoderAState = digitalRead(encoderPinA);
  int encoderBState = digitalRead(encoderPinB);
  
  if ((encoderAState == LOW) && (encoderBState == HIGH)) {
    encoderPos++;
  } else if ((encoderAState == HIGH) && (encoderBState == LOW)) {
    encoderPos--;
  }
}

void setup() {
  // Setup pin modes and initial logic signals
  pinMode(dirPin, OUTPUT); // Set direction pin as output
  pinMode(pulPin, OUTPUT); // Set pulse pin as output
  pinMode(encoderPinA, INPUT_PULLUP); // Set encoder pin A as input with internal pull-up resistor
  pinMode(encoderPinB, INPUT_PULLUP); // Set encoder pin B as input with internal pull-up resistor

  digitalWrite(dirPin, LOW); // Set direction pin low
  digitalWrite(pulPin, LOW); // Set pulse pin low

  // Start serial communication
  Serial.begin(9600);
  Serial.println("-------SYSTEM START------");

  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  // Calculate maximum number of pulses
  maxNbrPulses = revolutions * (STEPS_PER_REVOLUTION * NUMBER_TEETH_B) / NUMBER_TEETH_A;

  // Calculate pulse delay based on RPM
  pulseDelay = (1000000 * 60) / (RPM * STEPS_PER_REVOLUTION);
}

void loop() {
  unsigned long currentMicros = micros(); // Get the current time

  // Check if it's time to generate a pulse
  if (currentMicros - previousMicros >= pulseDelay) {
    pulse(); // Generate a pulse
    previousMicros = currentMicros; // Update the time of the last pulse
    nbrPulses++; // Increment the pulse counter
  }

  // Calculate RPM
  float dt = (currentMicros - previousMicros) / 1000000.0; // Time elapsed in seconds
  float actualRPM = (encoderPos * 60.0) / (STEPS_PER_REVOLUTION * dt);

  // Calculate linear speed (meters per second)
  float linearSpeed = actualRPM * 2 * PI * WHEEL_RADIUS / 60.0;

  //Output linear speed to serial monitor
  if(linearSpeed >= 4){  
  Serial.print("Linear Speed (m/s): ");
  Serial.println(linearSpeed);}


  // Check if the maximum number of pulses has been reached
  if (nbrPulses >= maxNbrPulses) {
    while (true) {} // Loop indefinitely
  }
}

