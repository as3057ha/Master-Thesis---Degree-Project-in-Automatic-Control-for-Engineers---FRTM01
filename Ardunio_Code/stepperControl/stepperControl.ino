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
    0.0,0.0025000000000000005,0.005000000000000001,0.0075000000000000015,0.010000000000000002,0.0125,0.015000000000000003,0.0175,0.020000000000000004,0.022500000000000003,0.025,0.027500000000000004,0.030000000000000006,0.0325,0.035,0.037500000000000006,0.04000000000000001,0.04250000000000001,0.045000000000000005,0.04750000000000001,0.05,0.052500000000000005,0.05500000000000001,0.05750000000000001,0.06000000000000001,0.0625,0.065,0.0675,0.07,0.07250000000000001,0.07500000000000001,0.07750000000000001,0.08000000000000002,0.08250000000000002,0.08500000000000002,0.08750000000000001,0.09000000000000001,0.09250000000000001,0.09500000000000001,0.09750000000000002,0.1,0.10250000000000002,0.10500000000000001,0.1075,0.11000000000000001,0.1125,0.11500000000000002,0.11750000000000001,0.12000000000000002,0.12250000000000001,0.125,0.12750000000000003,0.13,0.13250000000000003,0.135,0.1375,0.14,0.14250000000000002,0.14500000000000002,0.14750000000000002,0.15000000000000002,0.15250000000000002,0.15500000000000003,0.15750000000000003,0.16000000000000003,0.1625,0.16500000000000004,0.1675,0.17000000000000004,0.17250000000000001,0.17500000000000002,0.17750000000000002,0.18000000000000002,0.18250000000000002,0.18500000000000003,0.1875,0.19000000000000003,0.1925,0.19500000000000003,0.1975,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.1975,0.195,0.1925,0.19,0.1875,0.185,0.1825,0.18,0.17750000000000002,0.17500000000000002,0.17250000000000001,0.17,0.1675,0.165,0.1625,0.16,0.1575,0.155,0.1525,0.15000000000000002,0.14750000000000002,0.14500000000000002,0.14250000000000002,0.14,0.1375,0.135,0.1325,0.13,0.1275,0.125,0.1225,0.12,0.1175,0.11499999999999999,0.1125,0.11,0.1075,0.105,0.1025,0.1,0.09749999999999999,0.095,0.09250000000000001,0.09,0.08750000000000001,0.08499999999999999,0.0825,0.07999999999999999,0.0775,0.07500000000000001,0.07249999999999998,0.07,0.06749999999999998,0.065,0.0625,0.06,0.057499999999999996,0.05499999999999999,0.05249999999999999,0.04999999999999999,0.04749999999999999,0.044999999999999984,0.04249999999999998,0.03999999999999998,0.037500000000000006,0.034999999999999976,0.0325,0.02999999999999997,0.027499999999999997,0.024999999999999994,0.022499999999999992,0.01999999999999999,0.017499999999999988,0.014999999999999986,0.012500000000000011,0.009999999999999981,0.007500000000000007,0.004999999999999977,0.0025000000000000022,0.0
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


