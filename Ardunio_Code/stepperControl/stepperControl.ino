#include "FrequencyManager.h"

// Global pointer declaration
FrequencyManager* myFrequencyManager = nullptr;

// Define the connection pins
const int DIR_PIN = 8; // DIR- connected to pin 8
const int PUL_PIN = 9; // PUL- connected to pin 9

// Define motor control variables
const int STEPS_PER_REVOLUTION = 3200; // Set this to the number of pulses per revolution as per DIP switch settings

// Define physical variables
const double NUMBER_TEETH_A = 32;
const double NUMBER_TEETH_B = 60;
const double FEEDING_RADIUS = 0.05; // In meters 

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(PUL_PIN, LOW);

  // Calculate feedPerPulse based on the provided constants
  double feedPerPulse = calculateFeedPerPulse(NUMBER_TEETH_A, NUMBER_TEETH_B, STEPS_PER_REVOLUTION);
  
  // Initialize myFrequencyManager with the calculated value
  myFrequencyManager = new FrequencyManager(feedPerPulse);
}

double calculateFeedPerPulse(double teethA, double teethB, int stepsPerRevolution) {
  // Assuming the formula to calculate feed per pulse is correct based on the mechanical setup
  return (360.0 / stepsPerRevolution) * (teethA / teethB) / 360.0;
}

void loop() {
  // Placeholder for real feeding speed which will be variable later
  double feedingSpeed = 0.1; // This value will be dynamic in the future
  myFrequencyManager->update(feedingSpeed);

  // Perform a step at the frequency determined by the updated feeding speed
  performStepAtFrequency(myFrequencyManager->getFrequency());
}

void performStepAtFrequency(double frequency) {
  static unsigned long lastStepTime = 0;
  long pulseInterval = 1000000 / frequency; // Interval in microseconds

  unsigned long currentMicros = micros();
  if (currentMicros - lastStepTime >= pulseInterval) {
    makeSingleStep();
    lastStepTime = currentMicros;
  }
}

void makeSingleStep() {
  // Generates a pulse to step the motor
  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(5); // Maintain HIGH state for at least 5 microseconds
  digitalWrite(PUL_PIN, LOW);
}
