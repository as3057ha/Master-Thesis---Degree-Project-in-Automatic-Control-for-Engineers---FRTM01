#include <Stepper.h>

// Define the connection pins
const int DIR_PIN = 8; // DIR- connected to pin 8
const int PUL_PIN = 9; // PUL- connected to pin 9

// Define control variables
const double STEPS_PER_REVOLUTION = 800; // Set this to the number of pulses per revolution as per DIP switch settings
const double SAMPLE_TIME = 500; // In miliseconds

// Stepper definition
Stepper JMC(STEPS_PER_REVOLUTION,PUL_PIN,DIR_PIN);

// Define physical variables
const float NUMBER_TEETH_A = 32;
const float NUMBER_TEETH_B = 60;
const float FEEDING_RADIUS = (30.0 - 1.427)*0.001; // In meters 

// Velocity
float vTcp;
float rpm;
int steps;

// Mock data
float vectorData[] = {
    0.009359928372335902,0.03638185375370186,0.09325720705333092,0.13927239806346536,0.19323586187029457,0.2620708178503811,0.29315063982769296,0.3488900271237711,0.38079455255917793,0.445191544375395,0.5039192239572305,0.4988673183973963,0.510765491008229,0.5017196892340765,0.514772276314553,0.498938678758812,0.5107059696198987,0.4949068549209342,0.5065431825475957,0.5115436237837065,0.4961576336201521,0.49583723970585114,0.0024730499707425885
};
int currentIndex = 0;

int totalElements = sizeof(vectorData) / sizeof(vectorData[0]);


void setup() {
  //Serial communication for debugging
  Serial.begin(9600); 
  // Set DIR_PIN and PUL_PIN as outputs
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  
  // Set DIR_PIN low
  digitalWrite(DIR_PIN, HIGH);
  Serial.println("Setup Complete");
}

void loop() {
  Serial.println("-----------------------------");
  Serial.println("Current index: " + (String)currentIndex);

  // Get the current time
  unsigned long t0 = millis();
  // Read robot velocity
  vTcp = readVTcp();
  Serial.println("vTcp: " + (String)vTcp);
  // Calculate corrisponding rpm
  rpm = calculateRPM(vTcp);
  Serial.println("rpm: " + (String)rpm);
  // Set the motor speed 
  JMC.setSpeed(rpm);
  // Calculate the number of steps needed to maintain the sample time
  steps = calculateNbrSteps(rpm);
  // Step the motor
  Serial.println("Steps: " + (String)steps);
  JMC.step(steps);
  // Get the current time
  unsigned long t1 = millis();
  // Calculate the time passed in this loop iteration
  unsigned long tPassed = t1 - t0;
  Serial.println("Loop time in ml" + (String)tPassed);

 // Serial.println("Steps per s" + (String)((1/60)*steps*rpm));

  // // Check if the time passed is less than the desired sample time
  // if (tPassed < SAMPLE_TIME){
  //   // If so, delay for the remaining time to meet the sample time
  //   delay(SAMPLE_TIME- tPassed);
  // }
  
}

double readVTcp() {
    // Check if all elements have been read
    if (currentIndex >= totalElements) {
        // Stop the main loop
        while (true) {
            // Do nothing, just keep the main loop running
        }
    }

    // Return the current value and update the index for the next call
    float currentValue = vectorData[currentIndex];
    currentIndex++;
    return currentValue;
}

double calculateRPM(double vTcp) {
  return vTcp*60*NUMBER_TEETH_B/(FEEDING_RADIUS*2*PI*NUMBER_TEETH_A);}

int calculateNbrSteps(double rpm){

  // Debugging prints for each variable
  Serial.print("SAMPLE_TIME: ");
  Serial.println(SAMPLE_TIME);
  Serial.print("STEPS_PER_REVOLUTION: ");
  Serial.println(STEPS_PER_REVOLUTION);
  Serial.print("rpm: ");
  Serial.println(rpm);

  // Now perform the calculation
  float rpmOverTime = rpm / 60.0; // RPM to RPS (Revolutions per Second)
  float stepsPerSecond = STEPS_PER_REVOLUTION * rpmOverTime;
  float stepsPerMillisecond = stepsPerSecond / 1000.0; // Convert seconds to milliseconds
  float dnbrSteps = SAMPLE_TIME * stepsPerMillisecond;

  // Print the result of the calculation
  Serial.print("dnbrSteps (before casting): ");
  Serial.println(dnbrSteps);
  int nbrSteps = (int)dnbrSteps;
  return nbrSteps;
}


